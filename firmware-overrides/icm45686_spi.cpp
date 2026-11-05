/**
 * @file icm45686_spi.cpp
 * @brief Dual TDK InvenSense ICM-45686 IMU SPI Driver Implementation
 *
 * Register addresses, bit positions and the soft-reset / endianness handling
 * follow TDK's open-source driver (tdk-invn-oss/motion.arduino.ICM45686).
 */

#include "icm45686_spi.h"
#include <SPI.h>
#include <math.h>

namespace {
constexpr uint8_t kInt1Config0 = 0x16;
constexpr uint8_t kDataReadyBit = 0x04;
constexpr uint32_t kCalibrationTimeoutUs = 10000000;
constexpr uint32_t kCalibrationWindowUs = 1000000;

int16_t saturatedCount(float value) {
  if (value >= 32767.0f) return 32767;
  if (value <= -32768.0f) return -32768;
  return (int16_t)lroundf(value);
}
}


DualIcm45686 imu;

DualIcm45686::DualIcm45686() {
  bias1 = {0, 0, 0};
  bias2 = {0, 0, 0};
  _cs[0] = PIN_SPI_CS1;
  _cs[1] = PIN_SPI_CS2;
  _bigEndian[0] = _bigEndian[1] = false;
  _ok[0] = _ok[1] = false;
  _whoAmI[0] = _whoAmI[1] = 0;
  _lastFreshMask = 0;
  _calibrationState = ImuCalibrationState::Idle;
  _calibrationStartedUs = 0;
  resetCalibrationWindow(0);
}

void DualIcm45686::writeRegister(uint8_t csPin, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(csPin, LOW);
  SPI.transfer(reg & 0x7F); // Bit 7 = 0 for write
  SPI.transfer(val);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t DualIcm45686::readRegister(uint8_t csPin, uint8_t reg) {
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(csPin, LOW);
  SPI.transfer(reg | 0x80); // Bit 7 = 1 for read
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return val;
}

// Indirect (MREG) read of a single byte, as done in TDK's inv_imu_transport.c:
// write the 16-bit address to IREG_ADDR_15_8/7_0, then read IREG_DATA, with
// a few microseconds between accesses.
uint8_t DualIcm45686::readIndirect(uint8_t csPin, uint16_t addr) {
  delayMicroseconds(4);
  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(csPin, LOW);
  SPI.transfer(ICM_REG_IREG_ADDR_15_8 & 0x7F);
  SPI.transfer((uint8_t)(addr >> 8));
  SPI.transfer((uint8_t)(addr & 0xFF));
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  delayMicroseconds(4);
  return readRegister(csPin, ICM_REG_IREG_DATA);
}

bool DualIcm45686::initSensor(int idx) {
  const uint8_t cs = _cs[idx];

  // 1. Identify the chip (one retry, the first transaction after power-up can be lost)
  uint8_t who = readRegister(cs, ICM_REG_WHO_AM_I);
  if (who != ICM_WHO_AM_I_VALUE) {
    delay(5);
    who = readRegister(cs, ICM_REG_WHO_AM_I);
  }
  _whoAmI[idx] = who;
  if (who != ICM_WHO_AM_I_VALUE) {
    return false;
  }

  // 2. Preserve the two interface/drive registers that TDK's reference
  //    driver saves and restores around a soft reset.
  const uint8_t driveConfig0 = readRegister(cs, ICM_REG_DRIVE_CONFIG0);
  const uint8_t intfConfig1Ovrd = readRegister(cs, ICM_REG_INTF_CONFIG1_OVRD);

  // 3. Soft reset. REG_MISC2.SOFT_RST is bit 1 (0x02).
  writeRegister(cs, ICM_REG_MISC2, ICM_SOFT_RST_BIT);
  delay(1);  // TDK reference driver waits at least 1 ms for reset to take effect.

  // Restore the saved settings before using the sensor interface further.
  writeRegister(cs, ICM_REG_DRIVE_CONFIG0, driveConfig0);
  writeRegister(cs, ICM_REG_INTF_CONFIG1_OVRD, intfConfig1Ovrd);

  // 4. Read RESET_DONE. INT1_STATUS0 bit 7 is the RESET_DONE flag and is
  //    read-to-clear. The reset-done interrupt is enabled by the reset value
  //    of INT1_CONFIG0.
  const uint8_t resetStatus = readRegister(cs, ICM_REG_INT1_STATUS0);
  if ((resetStatus & ICM_RESET_DONE_BIT) == 0) {
    return false;
  }

  // 5. Read the device's actual data endianness. Reset default is little
  //    endian; the bit is 1 only if the host previously selected big endian.
  _bigEndian[idx] = (readIndirect(cs, ICM_MREG_SREG_CTRL) & ICM_SREG_BIG_ENDIAN_BIT) != 0;

  // 6. Full scale and output data rate while the sensors are still off.
  //    TDK's register definitions use ODR code 0x5 for 1600 Hz and gyro
  //    FS code 0x1 for +/-2000 dps; accel FS code 0x1 is +/-16 g.
  const uint8_t gyroCfg  = (uint8_t)((IMU_GYRO_FS << 4) | IMU_GYRO_ODR);
  const uint8_t accelCfg = (uint8_t)((IMU_ACCEL_FS << 4) | IMU_ACCEL_ODR);
  writeRegister(cs, ICM_REG_GYRO_CONFIG0, gyroCfg);
  writeRegister(cs, ICM_REG_ACCEL_CONFIG0, accelCfg);

  // Enable the data-ready status bit. INT1 need not be wired: the bit is
  // polled over SPI and read-to-clear (TDK INT1_CONFIG0/INT1_STATUS0 bit 2).
  writeRegister(cs, kInt1Config0,
                readRegister(cs, kInt1Config0) | kDataReadyBit);

  // 7. Accel + gyro into low-noise mode, then wait out the startup time
  //    (accel ~10 ms, gyro ~70 ms per TDK's driver)
  writeRegister(cs, ICM_REG_PWR_MGMT0, ICM_PWR_ACCEL_GYRO_LN);
  delay(100);

  // 8. Read back to confirm the configuration took
  if (readRegister(cs, ICM_REG_GYRO_CONFIG0) != gyroCfg) return false;
  if (readRegister(cs, ICM_REG_ACCEL_CONFIG0) != accelCfg) return false;
  if ((readRegister(cs, ICM_REG_PWR_MGMT0) & 0x0F) != ICM_PWR_ACCEL_GYRO_LN) return false;

  if (!(readRegister(cs, kInt1Config0) & kDataReadyBit)) return false;
  return true;
}

bool DualIcm45686::begin() {
  _ok[0] = _ok[1] = false;
  _lastFreshMask = 0;
  _calibrationState = ImuCalibrationState::Idle;
  // Set CS pins as OUTPUT and HIGH
  pinMode(PIN_SPI_CS1, OUTPUT);
  pinMode(PIN_SPI_CS2, OUTPUT);
  digitalWrite(PIN_SPI_CS1, HIGH);
  digitalWrite(PIN_SPI_CS2, HIGH);
  delay(10);

  // Initialize RP2040 SPI0 pins (chip selects are driven manually)
  SPI.setRX(PIN_SPI_MISO);
  SPI.setSCK(PIN_SPI_SCK);
  SPI.setTX(PIN_SPI_MOSI);
  SPI.begin();

  _ok[0] = initSensor(0);
  _ok[1] = initSensor(1);

  delay(50);
  return _ok[0] && _ok[1];
}

bool DualIcm45686::readSensorData(int idx, ImuRawData &data) {
  data = {0, 0, 0, 0, 0, 0};
  if (idx < 0 || idx > 1 || !_ok[idx]) return false;

  const uint8_t cs = _cs[idx];
  const uint8_t status = readRegister(cs, ICM_REG_INT1_STATUS0);
  // 0xff is a common disconnected-MISO response, not a trustworthy sample.
  if (status == 0xff || !(status & kDataReadyBit)) return false;
  uint8_t raw[12];

  SPI.beginTransaction(SPISettings(SPI_CLOCK_SPEED_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(ICM_REG_ACCEL_DATA_X1 | 0x80); // Burst read from 0x00
  for (int i = 0; i < 12; i++) {
    raw[i] = SPI.transfer(0x00);
  }
  digitalWrite(cs, HIGH);
  SPI.endTransaction();

  // Layout: accel X,Y,Z (6 bytes) then gyro X,Y,Z (6 bytes). Byte order depends on SREG_CTRL.
  int16_t v[6];
  for (int i = 0; i < 6; i++) {
    const uint8_t b0 = raw[2 * i];
    const uint8_t b1 = raw[2 * i + 1];
    v[i] = _bigEndian[idx] ? (int16_t)((b0 << 8) | b1) : (int16_t)((b1 << 8) | b0);
  }

  data.ax = v[0];
  data.ay = v[1];
  data.az = v[2];
  data.gx = v[3];
  data.gy = v[4];
  data.gz = v[5];
  return true;
}

bool DualIcm45686::readFusedMotion(ImuMotionData &fused) {
  ImuRawData data[2] = {};
  _lastFreshMask = 0;
  for (int i = 0; i < 2; ++i)
    if (readSensorData(i, data[i])) _lastFreshMask |= (1U << i);
  updateCalibration(data, _lastFreshMask, micros());

  fused = {};
  unsigned count = 0;
  for (int i = 0; i < 2; ++i) {
    if (!(_lastFreshMask & (1U << i))) continue;
    const ImuBiasOffsets bias = sensorBias(i);
    fused.gx += (float)data[i].gx - bias.gx;
    fused.gy += (float)data[i].gy - bias.gy;
    fused.gz += (float)data[i].gz - bias.gz;
    fused.ax += data[i].ax;
    fused.ay += data[i].ay;
    fused.az += data[i].az;
    ++count;
  }
  if (!count) return false;
  // Both devices must be mounted with matching sensor axes. A board transform
  // belongs at the DS4 boundary and must be identical for accel and gyro.
  fused.gx = fused.gx / count * IMU_SIGN_X;
  fused.gy = fused.gy / count * IMU_SIGN_Y;
  fused.gz = fused.gz / count * IMU_SIGN_Z;
  fused.ax = fused.ax / count * IMU_SIGN_X;
  fused.ay = fused.ay / count * IMU_SIGN_Y;
  fused.az = fused.az / count * IMU_SIGN_Z;
  return true;
}

bool DualIcm45686::readFusedMotion(ImuRawData &fused) {
  ImuMotionData motion;
  const bool fresh = readFusedMotion(motion);
  fused = {saturatedCount(motion.gx), saturatedCount(motion.gy),
           saturatedCount(motion.gz), saturatedCount(motion.ax),
           saturatedCount(motion.ay), saturatedCount(motion.az)};
  return fresh;
}

ImuBiasOffsets DualIcm45686::sensorBias(int idx) const {
  if (idx == 0) return bias1;
  if (idx == 1) return bias2;
  return {0.0f, 0.0f, 0.0f};
}

uint32_t DualIcm45686::calibrationSamples(int idx) const {
  return idx >= 0 && idx < 2 ? _calibration[idx].n : 0;
}

void DualIcm45686::resetCalibrationWindow(uint32_t now) {
  _windowStartedUs = now;
  for (int i = 0; i < 2; ++i) {
    _calibration[i] = {};
    for (int axis = 0; axis < 6; ++axis) {
      _calibration[i].minimum[axis] = 1.0e9f;
      _calibration[i].maximum[axis] = -1.0e9f;
    }
  }
}

void DualIcm45686::startCalibration() {
  _calibrationStartedUs = micros();
  resetCalibrationWindow(_calibrationStartedUs);
  _calibrationState = (_ok[0] || _ok[1]) ? ImuCalibrationState::Collecting
                                        : ImuCalibrationState::TimedOut;
}

bool DualIcm45686::calibrationSampleStill(const ImuRawData &data) const {
  const float gx = data.gx / IMU_COUNTS_PER_DPS;
  const float gy = data.gy / IMU_COUNTS_PER_DPS;
  const float gz = data.gz / IMU_COUNTS_PER_DPS;
  // Allow the sensor's zero-rate offset, but reject deliberate large rotation.
  if (gx * gx + gy * gy + gz * gz > 25.0f) return false;
  const float ax = data.ax / IMU_COUNTS_PER_G;
  const float ay = data.ay / IMU_COUNTS_PER_G;
  const float az = data.az / IMU_COUNTS_PER_G;
  const float gravitySquared = ax * ax + ay * ay + az * az;
  // Any resting orientation is valid. Never subtract gravity as an accel bias.
  return gravitySquared >= 0.85f * 0.85f && gravitySquared <= 1.15f * 1.15f;
}

bool DualIcm45686::calibrationWindowStill(int idx) const {
  const CalibrationWindow &w = _calibration[idx];
  if (w.n < CALIBRATION_SAMPLES || w.n < 2) return false;
  for (int axis = 0; axis < 6; ++axis) {
    const float scale = axis < 3 ? IMU_COUNTS_PER_DPS : IMU_COUNTS_PER_G;
    const float stddevLimit = (axis < 3 ? 0.25f : 0.015f) * scale;
    const float rangeLimit = (axis < 3 ? 2.0f : 0.06f) * scale;
    if (w.m2[axis] / (w.n - 1) > stddevLimit * stddevLimit ||
        w.maximum[axis] - w.minimum[axis] > rangeLimit) return false;
  }
  return true;
}

void DualIcm45686::updateCalibration(const ImuRawData data[2], uint8_t fresh,
                                   uint32_t now) {
  if (_calibrationState != ImuCalibrationState::Collecting) return;
  if ((uint32_t)(now - _calibrationStartedUs) >= kCalibrationTimeoutUs) {
    _calibrationState = ImuCalibrationState::TimedOut;
    return; // Never replace a previous good bias with an incomplete window.
  }
  for (int i = 0; i < 2; ++i) {
    if (!(fresh & (1U << i))) continue;
    if (!calibrationSampleStill(data[i])) {
      resetCalibrationWindow(now);
      return;
    }
    CalibrationWindow &w = _calibration[i];
    const float value[6] = {(float)data[i].gx, (float)data[i].gy,
                            (float)data[i].gz, (float)data[i].ax,
                            (float)data[i].ay, (float)data[i].az};
    ++w.n;
    for (int axis = 0; axis < 6; ++axis) {
      const float delta = value[axis] - w.mean[axis];
      w.mean[axis] += delta / w.n;
      w.m2[axis] += delta * (value[axis] - w.mean[axis]);
      if (value[axis] < w.minimum[axis]) w.minimum[axis] = value[axis];
      if (value[axis] > w.maximum[axis]) w.maximum[axis] = value[axis];
    }
  }
  if ((uint32_t)(now - _windowStartedUs) < kCalibrationWindowUs) return;
  for (int i = 0; i < 2; ++i)
    if (_ok[i] && _calibration[i].n < CALIBRATION_SAMPLES) return;
  for (int i = 0; i < 2; ++i) {
    if (_ok[i] && !calibrationWindowStill(i)) {
      resetCalibrationWindow(now);
      return;
    }
  }
  if (_ok[0]) bias1 = {_calibration[0].mean[0], _calibration[0].mean[1],
                      _calibration[0].mean[2]};
  if (_ok[1]) bias2 = {_calibration[1].mean[0], _calibration[1].mean[1],
                      _calibration[1].mean[2]};
  _calibrationState = ImuCalibrationState::Complete;
}

bool DualIcm45686::calibrateRestingOffsets() {
  startCalibration();
  while (_calibrationState == ImuCalibrationState::Collecting) {
    ImuMotionData unused;
    readFusedMotion(unused);
    delay(1); // Yield for background USB work; use the nonblocking API in loop().
  }
  return _calibrationState == ImuCalibrationState::Complete;
}
