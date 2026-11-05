#include "usb_ds4_hid.h"
#include <assert.h>
#include <cmath>
#include <cstdio>
#include <cstring>

using Hid = Adafruit_USBD_HID;

static int16_t signed16(const uint8_t* p) {
  return static_cast<int16_t>(uint16_t(p[0]) | uint16_t(p[1]) << 8);
}

static void descriptorContract() {
  uint32_t bits[4][256] = {};
  uint32_t size=0, count=0, id=0;
  for (size_t offset=0; offset<Hid::descriptor.size();) {
    const uint8_t prefix = Hid::descriptor[offset++];
    assert(prefix != 0xFE);
    const unsigned len = (prefix & 3) == 3 ? 4 : prefix & 3;
    assert(offset+len <= Hid::descriptor.size());
    uint32_t value = 0;
    for (unsigned i=0; i<len; ++i) value |= uint32_t(Hid::descriptor[offset++]) << (8*i);
    switch(prefix & 0xFC) {
      case 0x74: size=value; break;
      case 0x94: count=value; break;
      case 0x84: id=value; assert(id<256); break;
      case 0x80: bits[1][id] += size*count; break;
      case 0x90: bits[2][id] += size*count; break;
      case 0xB0: bits[3][id] += size*count; break;
    }
  }
  assert(bits[1][1] == 63*8);
  assert(bits[2][5] == 31*8);
  assert(bits[3][2] == 36*8);
  assert(bits[3][0x12] == 15*8);
  assert(bits[3][0xA3] == 48*8);
  unsigned reports=0;
  for (unsigned type=1; type<4; ++type)
    for (unsigned report=0; report<256; ++report) reports += bits[type][report] != 0;
  assert(reports == 5);
}

static void calibrationContract() {
  uint8_t p[64];
  memset(p,0xEE,sizeof(p));
  assert(Hid::getter(2,HID_REPORT_TYPE_FEATURE,p,sizeof(p)) == 36);
  assert(p[36] == 0xEE); // Does not include an extra report ID.
  for (unsigned i=0; i<3; ++i) {
    assert(signed16(p+2*i) == 0);
    const float speed = signed16(p+18) + signed16(p+20);
    const float counts = signed16(p+6+4*i) - signed16(p+8+4*i);
    // Apply Linux's feature-based calibration to a known sample, and compare
    // with the independent fixed scale used by JoyShockLibrary.
    assert(std::fabs(10000*speed/counts - 10000*2000.0f/32767.0f) < 0.001f);
    const int plus = signed16(p+22+4*i), minus = signed16(p+24+4*i);
    assert(plus-minus == 16384);
    assert(plus-(plus-minus)/2 == 0);
    assert(8192*2.0f/(plus-minus) == 1.0f);
  }
  memset(p,0xEE,sizeof(p));
  assert(Hid::getter(2,HID_REPORT_TYPE_FEATURE,p,7) == 7);
  assert(p[7] == 0xEE);
  assert(Hid::getter(0x12,HID_REPORT_TYPE_FEATURE,p,sizeof(p)) == 15);
  assert((p[5]&3) == 2); // Local unicast board identity.
  assert(Hid::getter(0xA3,HID_REPORT_TYPE_FEATURE,p,sizeof(p)) == 48);
  assert(signed16(p+34) != 0 && signed16(p+40) != 0);
  assert(Hid::getter(0x99,HID_REPORT_TYPE_FEATURE,p,sizeof(p)) == 0);
  assert(Hid::getter(2,HID_REPORT_TYPE_OUTPUT,p,sizeof(p)) == 0);
}

static void wireContract() {
  UsbDs4Hid usb;
  DS4InputReport report;
  usb.initializeReport(report);
  report.gyro_x=0x1234; report.gyro_y=-1234; report.gyro_z=32767;
  report.accel_x=8192; report.accel_y=-8192; report.accel_z=4096;
  usb.prepareReport(report, 10000, false);
  assert(usb.sendReport(report));
  assert(Hid::packet.size() == 64 && Hid::packet[0] == 1);
  const auto* b=Hid::packet.data();
  assert(b[1]==128 && b[4]==128 && b[5]==8);
  assert(b[12]==0); // Temperature, never battery.
  assert(signed16(b+13)==0x1234 && signed16(b+15)==-1234 && signed16(b+17)==32767);
  assert(signed16(b+19)==8192 && signed16(b+21)==-8192 && signed16(b+23)==4096);
  assert(b[30]==0x1B && b[31]==0 && b[33]==1);
  assert((b[35]&0x80) && (b[39]&0x80));
  assert((b[36] | (b[37]&15)<<8)==960);
  assert(((b[37]>>4) | b[38]<<4)==471);
  uint8_t copy[63];
  assert(Hid::getter(1,HID_REPORT_TYPE_INPUT,copy,sizeof(copy)) == 63);
  assert(memcmp(copy,b+1,63)==0);

  usb.prepareReport(report, 11000, true);
  assert(report.buttons3 == 6); // First accepted packet advanced sequence.
  const uint8_t contact=report.finger1_id;
  assert((contact&0x80)==0);
  assert(usb.sendReport(report));
  usb.prepareReport(report, 12000, true);
  assert(report.finger1_id==contact); // Not a new finger every millisecond.
  Hid::endpointReady=false;
  const unsigned oldCalls=Hid::queueCalls;
  assert(!usb.sendReport(report) && Hid::queueCalls==oldCalls);
  Hid::endpointReady=true;
  Hid::acceptSend=false;
  assert(!usb.sendReport(report));
  Hid::acceptSend=true;
  usb.prepareReport(report, 13000, false);
  assert((report.buttons3>>2)==2); // Both failed attempts preserve sequence.
  assert(report.finger1_id==(contact|0x80));
  assert(usb.sendReport(report));
  usb.prepareReport(report, 14000, true);
  assert(report.finger1_id==uint8_t(contact+1));
  assert(report.finger2_id==0x80);
}

static void clockContract() {
  UsbDs4Hid usb;
  DS4InputReport report;
  usb.initializeReport(report);
  const uint32_t start=0xFFFFF000;
  usb.prepareReport(report,start,false);
  for (uint32_t i=1; i<=100000; ++i) {
    const uint16_t previous=report.timestamp;
    usb.prepareReport(report,start+i*1000,false);
    const uint16_t step=report.timestamp-previous;
    assert(step==187 || step==188);
    assert(report.timestamp==uint16_t((uint64_t(i)*1000*3)/16));
  }
  // Variable intervals preserve fractional ticks as well.
  UsbDs4Hid jitter;
  jitter.initializeReport(report);
  uint32_t now=0; uint64_t elapsed=0;
  jitter.prepareReport(report,now,false);
  for (unsigned i=1; i<5000; ++i) {
    const uint32_t dt=250+(i%29)*111;
    now+=dt; elapsed+=dt;
    jitter.prepareReport(report,now,false);
    assert(report.timestamp==uint16_t(elapsed*3/16));
  }
}

int main() {
  usbDs4.begin();
  assert(TinyUSBDevice.detached && TinyUSBDevice.attached);
  assert(TinyUSBDevice.vid==0x054C && TinyUSBDevice.pid==0x05C4);
  assert(Hid::interval==1);
  assert(Hid::getter && Hid::setter);
  descriptorContract();
  calibrationContract();
  wireContract();
  clockContract();
  puts("PASS: DS4 descriptor, calibration, wire layout, touch, queue counters, timestamp rollover");
}
