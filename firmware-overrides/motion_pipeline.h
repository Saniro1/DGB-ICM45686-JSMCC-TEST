#ifndef MOTION_PIPELINE_H_
#define MOTION_PIPELINE_H_
#include <math.h>
#include <stdint.h>

struct MotionVector { float x, y, z; };
// Preserve the last source test's proper rotation for BOTH gyro and gravity:
// +X -> +X, +Y -> +Z, +Z -> -Y. The two IMUs must be mounted alike.
inline MotionVector sensorToController(MotionVector v) { return {v.x, -v.z, v.y}; }
inline int16_t motionCount(float v) {
  if (!isfinite(v)) return 0;
  if (v >= 32767.0f) return 32767;
  if (v <= -32768.0f) return -32768;
  return static_cast<int16_t>(lroundf(v));
}
// Units expected by JoyShockLibrary's DS4 parser.
inline MotionVector ds4Gyro(MotionVector v, float countsPerDps) {
  const float k = (32767.0f / 2000.0f) / countsPerDps;
  return sensorToController({v.x*k, v.y*k, v.z*k});
}
inline MotionVector ds4Accel(MotionVector v, float countsPerG) {
  const float k = 8192.0f / countsPerG;
  return sensorToController({v.x*k, v.y*k, v.z*k});
}
// Fixed 1 ms deadlines. Failed USB submissions retain the deadline for retry.
// Unsigned elapsed time handles micros() rollover. Never catch up in bursts.
class ReportSchedule {
public:
  void reset(uint32_t now) { last_ = now; }
  bool due(uint32_t now) const { return uint32_t(now - last_) >= 1000U; }
  void sent(uint32_t now) {
    last_ = uint32_t(now - last_) >= 2000U ? now : last_ + 1000U;
  }
private:
  uint32_t last_ = 0;
};
#endif

