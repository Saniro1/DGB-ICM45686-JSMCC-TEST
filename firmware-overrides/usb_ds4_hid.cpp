#include "usb_ds4_hid.h"
#include <string.h>
#include <pico/unique_id.h>

UsbDs4Hid usbDs4;

// Conventional DS4 USB gamepad descriptor. Motion and touch remain vendor
// bytes, parsed by DS4-aware hosts. The hat/buttons must also be exposed to
// generic HID clients rather than hiding all three button bytes as vendor data.
static const uint8_t ds4_hid_report_descriptor[] = {
  0x05,0x01, 0x09,0x05, 0xA1,0x01, 0x85,0x01,
  0x09,0x30, 0x09,0x31, 0x09,0x32, 0x09,0x35,
  0x15,0x00, 0x26,0xFF,0x00, 0x75,0x08, 0x95,0x04, 0x81,0x02,
  0x09,0x39, 0x15,0x00, 0x25,0x07, 0x35,0x00, 0x46,0x3B,0x01,
  0x65,0x14, 0x75,0x04, 0x95,0x01, 0x81,0x42,
  0x65,0x00, 0x05,0x09, 0x19,0x01, 0x29,0x0E,
  0x15,0x00, 0x25,0x01, 0x75,0x01, 0x95,0x0E, 0x81,0x02,
  0x06,0x00,0xFF, 0x09,0x20, 0x15,0x00, 0x25,0x3F,
  0x75,0x06, 0x95,0x01, 0x81,0x02,
  0x05,0x01, 0x09,0x33, 0x09,0x34, 0x15,0x00, 0x26,0xFF,0x00,
  0x75,0x08, 0x95,0x02, 0x81,0x02,
  0x06,0x00,0xFF, 0x09,0x21, 0x95,0x36, 0x81,0x02,
  // Output report (LED/rumble requests accepted, no actuators on this board).
  0x85,0x05, 0x09,0x22, 0x95,0x1F, 0x91,0x02,
  // USB calibration, pairing identity and firmware information.
  0x85,0x02, 0x09,0x24, 0x95,0x24, 0xB1,0x02,
  0x85,0x12, 0x09,0x2A, 0x95,0x0F, 0xB1,0x02,
  0x85,0xA3, 0x09,0x43, 0x95,0x30, 0xB1,0x02,
  0xC0
};

static DS4InputReport lastAcceptedReport;
static uint8_t pairingIdentity[6];

static void put16(uint8_t* p, int16_t value) {
  const uint16_t bits = static_cast<uint16_t>(value);
  p[0] = static_cast<uint8_t>(bits);
  p[1] = static_cast<uint8_t>(bits >> 8);
}

// TinyUSB inserts the report ID itself: all callback buffers exclude it.
static uint16_t getReport(uint8_t id, hid_report_type_t type,
                          uint8_t* buffer, uint16_t requested) {
  if (type == HID_REPORT_TYPE_INPUT && id == 1) {
    const uint16_t n = requested < sizeof(lastAcceptedReport)
                           ? requested : sizeof(lastAcceptedReport);
    memcpy(buffer, &lastAcceptedReport, n);
    return n;
  }
  if (type != HID_REPORT_TYPE_FEATURE) return 0;

  uint8_t payload[48] = {};
  uint16_t length;
  switch (id) {
    case 0x02:
      length = 36;
      // Firmware has already removed each IMU's zero-rate bias and normalized
      // samples to DS4/JoyShockLibrary units: gyro 32767/2000 counts per dps,
      // accel 8192 counts per g. Describe those units, not unmeasured factory
      // offsets. USB order is biasXYZ, +X,-X,+Y,-Y,+Z,-Z, speeds, accel pairs.
      for (unsigned i = 0; i < 3; ++i) {
        put16(payload + 6 + 4*i, 32767);
        put16(payload + 8 + 4*i, -32767);
        put16(payload + 22 + 4*i, 8192);
        put16(payload + 24 + 4*i, -8192);
      }
      put16(payload + 18, 2000);
      put16(payload + 20, 2000);
      break;
    case 0x12:
      length = 15;
      // Stable, locally administered board identity. This is a USB-only device;
      // no Bluetooth radio or pairing service is claimed.
      memcpy(payload, pairingIdentity, sizeof(pairingIdentity));
      break;
    case 0xA3:
      length = 48;
      memcpy(payload, "Nov  5 2026", 11);
      memcpy(payload + 16, "00:00:00", 8);
      put16(payload + 34, 0x0100);
      put16(payload + 40, 0x0100);
      break;
    default:
      return 0; // Unsupported requests stall instead of returning fake data.
  }
  const uint16_t n = requested < length ? requested : length;
  memcpy(buffer, payload, n);
  return n;
}

static void setReport(uint8_t, hid_report_type_t, const uint8_t*, uint16_t) {
  // DS4 output report 0x05 controls actuators absent from this board.
}

void UsbDs4Hid::initializeReport(DS4InputReport& report) {
  memset(&report, 0, sizeof(report));
  report.lx = report.ly = report.rx = report.ry = 128;
  report.buttons1 = 0x08; // Neutral hat.
  report.battery = 0x1B; // USB cable present, full; not the temperature field.
  report.touch_report_count = 1;
  report.finger1_id = report.finger2_id = 0x80; // No phantom second contact.
  // Binary touch electrode represented at the center of the DS4 touchpad.
  constexpr uint16_t x = 960, y = 471;
  report.finger1_x_low = x & 0xFF;
  report.finger1_x_high_y_low = (x >> 8) | ((y & 0x0F) << 4);
  report.finger1_y_high = y >> 4;
}

void UsbDs4Hid::begin() {
  // Registration can race initial enumeration on RP2040. Disconnect before
  // changing the descriptors and always reconnect after registration.
  TinyUSBDevice.detach();
  TinyUSBDevice.setID(0x054C, 0x05C4);
  TinyUSBDevice.setDeviceVersion(0x0101);
  TinyUSBDevice.setManufacturerDescriptor("DGB");
  TinyUSBDevice.setProductDescriptor("Dual ICM-45686 DS4");
  pico_unique_board_id_t boardId;
  pico_get_unique_board_id(&boardId);
  memcpy(pairingIdentity, boardId.id + PICO_UNIQUE_BOARD_ID_SIZE_BYTES - 6, 6);
  pairingIdentity[5] = (pairingIdentity[5] & 0xFC) | 0x02;
  initializeReport(lastAcceptedReport);
  _usbHid.setPollInterval(1); // Full-speed interrupt bInterval = one USB frame.
  _usbHid.setReportDescriptor(ds4_hid_report_descriptor,
                              sizeof(ds4_hid_report_descriptor));
  _usbHid.setReportCallback(getReport, setReport);
  _usbHid.begin();
  delay(10);
  TinyUSBDevice.attach();
}

void UsbDs4Hid::prepareReport(DS4InputReport& report, uint32_t sampleUs,
                             bool touched) {
  if (_clockInitialized) {
    // Unsigned subtraction survives micros() rollover. Keep fractional ticks
    // so 1000 us becomes alternating 187/188 ticks without cumulative error.
    const uint32_t elapsed = sampleUs - _lastSampleUs;
    const uint64_t scaled = uint64_t(elapsed) * 3 + _clockRemainder;
    _timestamp = static_cast<uint16_t>(_timestamp + scaled / 16);
    _clockRemainder = scaled % 16;
  } else {
    _clockInitialized = true;
  }
  _lastSampleUs = sampleUs;
  report.timestamp = _timestamp;
  report.buttons3 = (report.buttons3 & 0x01) | (touched ? 0x02 : 0) |
                    ((_sequence & 0x3F) << 2);
  report.touch_track_num = _sequence;
  // Keep the contact ID constant through a touch; increment only on touch-down.
  if (touched && !_wasTouched) _contactId = (_contactId + 1) & 0x7F;
  _wasTouched = touched;
  report.finger1_id = _contactId | (touched ? 0 : 0x80);
  report.finger2_id = 0x80;
}

bool UsbDs4Hid::isReady() { return _usbHid.ready(); }

bool UsbDs4Hid::sendReport(const DS4InputReport& report) {
  if (!_usbHid.ready() || !_usbHid.sendReport(1, &report, sizeof(report))) {
    return false;
  }
  lastAcceptedReport = report;
  ++_sequence; // Failed queue attempts must not advance the packet sequence.
  return true;
}
