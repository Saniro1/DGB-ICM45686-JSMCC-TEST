#ifndef USB_DS4_HID_H_
#define USB_DS4_HID_H_

#include <stddef.h>
#include <stdint.h>
#include <Adafruit_TinyUSB.h>

// USB report 0x01, excluding the one-byte report ID supplied by TinyUSB.
// Offsets follow Linux hid-playstation's dualshock4_input_report_usb.
struct DS4InputReport {
  uint8_t lx, ly, rx, ry;
  uint8_t buttons1, buttons2, buttons3;
  uint8_t l2_trigger, r2_trigger;
  uint16_t timestamp; // DS4 sensor clock: one tick = 16/3 microseconds.
  uint8_t temperature;
  int16_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;
  uint8_t reserved_motion[5];
  uint8_t battery; // Status0: cable present (bit4), capacity (bits0..3).
  uint8_t status1, reserved_status;
  uint8_t touch_report_count;
  uint8_t touch_track_num;
  uint8_t finger1_id, finger1_x_low, finger1_x_high_y_low, finger1_y_high;
  uint8_t finger2_id, finger2_x_low, finger2_x_high_y_low, finger2_y_high;
  uint8_t reserved[21]; // Two unused touch records (18) and trailing bytes (3).
} __attribute__((packed));

static_assert(sizeof(DS4InputReport) == 63, "DS4 HID payload must be 63 bytes");
static_assert(offsetof(DS4InputReport, timestamp) == 9, "DS4 timestamp offset");
static_assert(offsetof(DS4InputReport, gyro_x) == 12, "DS4 gyro offset");
static_assert(offsetof(DS4InputReport, accel_x) == 18, "DS4 accelerometer offset");
static_assert(offsetof(DS4InputReport, battery) == 29, "DS4 status offset");
static_assert(offsetof(DS4InputReport, touch_report_count) == 32, "DS4 touch count offset");
static_assert(offsetof(DS4InputReport, finger1_id) == 34, "DS4 first contact offset");
static_assert(offsetof(DS4InputReport, finger2_id) == 38, "DS4 second contact offset");

class UsbDs4Hid {
 public:
  void begin();
  static void initializeReport(DS4InputReport& report);
  void prepareReport(DS4InputReport& report, uint32_t sampleUs, bool touched);
  bool sendReport(const DS4InputReport& report);
  bool isReady();

 private:
  Adafruit_USBD_HID _usbHid;
  uint32_t _lastSampleUs = 0;
  uint16_t _timestamp = 0;
  uint8_t _clockRemainder = 0;
  uint8_t _sequence = 0;
  uint8_t _contactId = 0;
  bool _clockInitialized = false;
  bool _wasTouched = false;
};
extern UsbDs4Hid usbDs4;
#endif
