#pragma once
#include <stdint.h>
#include <vector>

enum hid_report_type_t { HID_REPORT_TYPE_INPUT = 1, HID_REPORT_TYPE_OUTPUT, HID_REPORT_TYPE_FEATURE };
struct UsbDeviceStub {
  bool detached = false, attached = false;
  uint16_t vid = 0, pid = 0, version = 0;
  void detach() { detached = true; }
  void attach() { attached = true; }
  void setID(uint16_t v, uint16_t p) { vid = v; pid = p; }
  void setDeviceVersion(uint16_t v) { version = v; }
  void setManufacturerDescriptor(const char*) {}
  void setProductDescriptor(const char*) {}
};
inline UsbDeviceStub TinyUSBDevice;
inline void delay(unsigned) {}

class Adafruit_USBD_HID {
 public:
  using Getter = uint16_t (*)(uint8_t, hid_report_type_t, uint8_t*, uint16_t);
  using Setter = void (*)(uint8_t, hid_report_type_t, const uint8_t*, uint16_t);
  inline static uint8_t interval = 0;
  inline static std::vector<uint8_t> descriptor, packet;
  inline static Getter getter = nullptr;
  inline static Setter setter = nullptr;
  inline static bool endpointReady = true, acceptSend = true;
  inline static unsigned queueCalls = 0;
  void setPollInterval(uint8_t n) { interval = n; }
  void setReportDescriptor(const uint8_t* p, uint16_t n) { descriptor.assign(p,p+n); }
  void setReportCallback(Getter g, Setter s) { getter = g; setter = s; }
  bool begin() { return true; }
  bool ready() { return endpointReady; }
  bool sendReport(uint8_t id, const void* data, uint8_t size) {
    ++queueCalls;
    if (!acceptSend) return false;
    packet.assign(1, id);
    const auto* p = static_cast<const uint8_t*>(data);
    packet.insert(packet.end(), p, p+size);
    return true;
  }
};
