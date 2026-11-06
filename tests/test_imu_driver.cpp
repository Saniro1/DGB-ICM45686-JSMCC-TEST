#include "icm45686_spi.h"
#include <SPI.h>
#include <assert.h>
#include <cmath>
#include <iostream>

FakeImuChip fakeChips[2];
FakeSpi SPI;
static uint32_t clockUs = 0;

void pinMode(uint8_t, int) {}
void digitalWrite(uint8_t pin, int level) {
  const int chip = pin == PIN_SPI_CS1 ? 0 : pin == PIN_SPI_CS2 ? 1 : -1;
  assert(chip >= 0);
  if (level == LOW) {
    assert(SPI.selected == -1); // Never activate both chip selects.
    SPI.selected = chip;
  } else if (SPI.selected == chip) SPI.selected = -1;
}
void delay(unsigned long ms) { clockUs += (uint32_t)ms * 1000; }
void delayMicroseconds(unsigned int us) { clockUs += us; }
uint32_t micros() { return clockUs; }
SPISettings::SPISettings(unsigned long clock, int order, int mode) {
  assert(clock == 6000000 && order == MSBFIRST && mode == SPI_MODE3);
}
void FakeSpi::beginTransaction(SPISettings) { command = true; }
void FakeSpi::endTransaction() { assert(selected == -1); }
uint8_t FakeSpi::transfer(uint8_t value) {
  assert(selected >= 0 && selected < 2);
  FakeImuChip &chip = fakeChips[selected];
  if (command) {
    command = false;
    reading = (value & 0x80) != 0;
    address = value & 0x7f;
    return 0;
  }
  const uint8_t reg = address++;
  if (reading) {
    if (reg == 0x19) {
      const uint8_t value = chip.registers[reg] |
          ((chip.ready && (chip.registers[0x16] & 4)) ? 4 : 0);
      chip.registers[reg] = 0;
      chip.ready = false;
      return value;
    }
    if (reg == 0x7e) {
      assert(chip.registers[0x7c] == 0xa2 && chip.registers[0x7d] == 0x67);
      return chip.bigEndian ? 2 : 0;
    }
    return chip.registers[reg];
  }
  chip.registers[reg] = value;
  if (reg == 0x7f && value == 2) chip.registers[0x19] = 0x80;
  return 0;
}

static void resetBus(bool secondPresent = true) {
  fakeChips[0] = {};
  fakeChips[1] = {};
  fakeChips[0].registers[0x72] = 0xe9;
  fakeChips[1].registers[0x72] = secondPresent ? 0xe9 : 0;
  fakeChips[1].bigEndian = true;
  SPI.selected = -1;
}
static void feed(int chip, int16_t gx, int16_t gy, int16_t gz,
                 int16_t ax = 0, int16_t ay = 0, int16_t az = 2048) {
  FakeImuChip &c = fakeChips[chip];
  const int16_t values[6] = {ax, ay, az, gx, gy, gz};
  for (int i = 0; i < 6; ++i) {
    const uint16_t v = (uint16_t)values[i];
    c.registers[2*i] = c.bigEndian ? v >> 8 : v & 255;
    c.registers[2*i+1] = c.bigEndian ? v & 255 : v >> 8;
  }
  c.ready = true;
}
static void near(float actual, float expected, float epsilon = 0.002f) {
  assert(std::fabs(actual - expected) < epsilon);
}
static void calibrate(DualIcm45686 &driver, bool second = true) {
  driver.startCalibration();
  ImuMotionData sample;
  for (int n = 0; n < 2000; ++n) {
    clockUs += 1000;
    feed(0, 10 + n % 2, -20, 6, 2048, 0, 0); // Resting on X face.
    if (second) feed(1, -30, 40 + n % 2, -8, 2048, 0, 0);
    assert(driver.readFusedMotion(sample));
  }
  assert(driver.calibrationState() == ImuCalibrationState::Complete);
  near(driver.bias1.gx, 10.5f);
  near(driver.bias1.gy, -20.0f);
  if (second) near(driver.bias2.gy, 40.5f);
  near(sample.ax, 2048.0f); // Preserve gravity; do not zero accelerometer.
}

int main() {
  resetBus();
  DualIcm45686 driver;
  assert(driver.begin());
  assert(driver.sensorOk(0) && driver.sensorOk(1));
  assert(!driver.sensorOk(-1) && !driver.sensorOk(2));
  assert((fakeChips[0].registers[0x16] & 4) != 0);
  feed(0, -1234, 5678, -90, 321, -654, 2048);
  feed(1, -1234, 5678, -90, 321, -654, 2048);
  ImuRawData raw;
  assert(driver.readSensorData(0, raw));
  assert(raw.gx == -1234 && raw.gy == 5678 && raw.ax == 321 && raw.ay == -654);
  assert(driver.readSensorData(1, raw)); // Big-endian path.
  assert(raw.gx == -1234 && raw.gz == -90 && raw.az == 2048);
  assert(!driver.readSensorData(1, raw)); // Already consumed data-ready.
  assert(!driver.readSensorData(-1, raw));
  assert(!driver.readSensorData(2, raw));

  calibrate(driver);
  ImuMotionData motion;
  feed(0, 11, -20, 6);
  feed(1, -30, 40, -8);
  assert(driver.readFusedMotion(motion));
  near(motion.gx, 0.25f); // Fractional per-sensor biases survive averaging.
  near(motion.gy, -0.25f);
  assert(driver.lastFreshMask() == 3);
  assert(!driver.readFusedMotion(motion));
  near(motion.gx, 0);
  assert(driver.lastFreshMask() == 0);

  // Recalibration during rotation must fail and keep the previous good bias.
  driver.startCalibration();
  for (int n = 0; n < 10000; ++n) {
    clockUs += 1000;
    feed(0, 200, 0, 0);
    feed(1, 0, 200, 0);
    driver.readFusedMotion(motion);
  }
  assert(driver.calibrationState() == ImuCalibrationState::TimedOut);
  near(driver.bias1.gx, 10.5f);
  near(driver.bias2.gy, 40.5f);

  // Gravity magnitude alone cannot establish rest: reject changing direction.
  driver.startCalibration();
  for (int n = 0; n < 2500; ++n) {
    clockUs += 1000;
    feed(0, 0, 0, 0, n % 2 ? 100 : -100, 0, 2046);
    feed(1, 0, 0, 0, n % 2 ? 100 : -100, 0, 2046);
    driver.readFusedMotion(motion);
  }
  assert(driver.calibrationState() == ImuCalibrationState::Collecting);
  assert(driver.calibrationSamples(0) < 1000);
  near(driver.bias1.gx, 10.5f);

  // No data-ready events cannot manufacture a successful calibration.
  driver.startCalibration();
  for (int n = 0; n < 10000; ++n) {
    clockUs += 1000;
    assert(!driver.readFusedMotion(motion));
  }
  assert(driver.calibrationState() == ImuCalibrationState::TimedOut);
  assert(driver.calibrationSamples(0) == 0);

  // Micros wrap must not break the calibration duration checks.
  clockUs = UINT32_MAX - 500000;
  calibrate(driver);

  // Corrected extremes saturate instead of wrapping to the opposite direction.
  driver.bias1.gx = -10000;
  feed(0, 32767, 0, 0);
  assert(driver.readFusedMotion(raw));
  assert(raw.gx == 32767);
  driver.bias1.gx = 10000;
  feed(0, -32768, 0, 0);
  assert(driver.readFusedMotion(raw));
  assert(raw.gx == -32768);

  // A missing second sensor is never averaged into the remaining sensor.
  resetBus(false);
  DualIcm45686 single;
  assert(!single.begin());
  assert(single.sensorOk(0) && !single.sensorOk(1));
  calibrate(single, false);
  feed(0, 11, -20, 6);
  assert(single.readFusedMotion(motion));
  near(motion.gx, 0.5f);
  near(motion.az, 2048);
  assert(single.lastFreshMask() == 1);
  std::cout << "IMU tests passed: endian/data-ready, dual and single fusion, fractional bias, "
               "stationary calibration, motion rejection, timeout, wrap and saturation.\\n";
}
