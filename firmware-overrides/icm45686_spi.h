#ifndef ICM45686_SPI_H_
#define ICM45686_SPI_H_

#include "config.h"

// Scales follow TDK's full-scale selector tables, not the rounded 16.4 figure.
static_assert(IMU_GYRO_FS >= 0 && IMU_GYRO_FS <= 8, "Unsupported gyro range");
static_assert(IMU_ACCEL_FS >= 0 && IMU_ACCEL_FS <= 4, "Unsupported accel range");
constexpr float IMU_COUNTS_PER_DPS = 32768.0f * (1U << IMU_GYRO_FS) / 4000.0f;
constexpr float IMU_COUNTS_PER_G = 32768.0f * (1U << IMU_ACCEL_FS) / 32.0f;

struct ImuRawData { int16_t gx, gy, gz, ax, ay, az; };
struct ImuMotionData { float gx, gy, gz, ax, ay, az; };
struct ImuBiasOffsets { float gx, gy, gz; };
enum class ImuCalibrationState { Idle, Collecting, Complete, TimedOut };

class DualIcm45686 {
public:
  DualIcm45686();
  bool begin();
  void writeRegister(uint8_t csPin, uint8_t reg, uint8_t val);
  uint8_t readRegister(uint8_t csPin, uint8_t reg);

  // Returns only new samples: reading a stale register is not a calibration sample.
  bool readSensorData(int idx, ImuRawData &data);
  bool readFusedMotion(ImuMotionData &fused);
  bool readFusedMotion(ImuRawData &fused); // Saturated compatibility conversion.

  // Call readFusedMotion regularly while collecting. All initialized IMUs must
  // pass before bias is replaced. Motion resets the window; failure retains bias.
  void startCalibration();
  ImuCalibrationState calibrationState() const { return _calibrationState; }
  uint32_t calibrationSamples(int idx) const;
  bool calibrateRestingOffsets(); // Blocking compatibility API; not for the HID loop.

  bool sensorOk(int idx) const { return idx >= 0 && idx < 2 && _ok[idx]; }
  uint8_t whoAmI(int idx) const { return idx >= 0 && idx < 2 ? _whoAmI[idx] : 0; }
  ImuBiasOffsets sensorBias(int idx) const;
  uint8_t lastFreshMask() const { return _lastFreshMask; }

  ImuBiasOffsets bias1, bias2;

private:
  struct CalibrationWindow {
    uint32_t n;
    float mean[6], m2[6], minimum[6], maximum[6];
  };
  bool initSensor(int idx);
  uint8_t readIndirect(uint8_t csPin, uint16_t addr);
  void resetCalibrationWindow(uint32_t now);
  void updateCalibration(const ImuRawData data[2], uint8_t fresh, uint32_t now);
  bool calibrationSampleStill(const ImuRawData &data) const;
  bool calibrationWindowStill(int idx) const;

  uint8_t _cs[2], _whoAmI[2], _lastFreshMask;
  bool _bigEndian[2], _ok[2];
  ImuCalibrationState _calibrationState;
  uint32_t _calibrationStartedUs, _windowStartedUs;
  CalibrationWindow _calibration[2];
};

extern DualIcm45686 imu;
#endif
