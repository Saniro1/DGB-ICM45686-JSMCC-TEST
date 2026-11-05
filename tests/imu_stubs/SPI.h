#pragma once
#include <stdint.h>
constexpr int MSBFIRST = 1, SPI_MODE3 = 3;
struct SPISettings {
  SPISettings(unsigned long clock, int order, int mode);
};
struct FakeImuChip {
  uint8_t registers[128] = {};
  bool bigEndian = false;
  bool ready = false;
};
extern FakeImuChip fakeChips[2];
class FakeSpi {
public:
  int selected = -1;
  bool command = true, reading = false;
  uint8_t address = 0;
  void beginTransaction(SPISettings);
  void endTransaction();
  uint8_t transfer(uint8_t value);
  void setRX(int) {}
  void setSCK(int) {}
  void setTX(int) {}
  void begin() {}
};
extern FakeSpi SPI;
