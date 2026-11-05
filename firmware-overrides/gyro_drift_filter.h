/**
 * Symmetric, low-latency gyro noise filtering.
 *
 * Bias must be measured per sensor during explicit stationary calibration,
 * before this stage. Constant slow rotation (especially yaw about gravity)
 * cannot be distinguished from gyro bias using IMU data alone. Learning bias
 * here used to erase intentional motion and leave an opposite residual when
 * the motion stopped. This filter has unity DC gain and no deadzone.
 *
 * Inputs and outputs use the same units (normally ICM-45686 gyro counts).
 */
#ifndef GYRO_DRIFT_FILTER_H_
#define GYRO_DRIFT_FILTER_H_

#include <math.h>
#include <stdint.h>

#ifndef GDF_CUTOFF_HZ
#define GDF_CUTOFF_HZ 80.0f
#endif

static_assert(GDF_CUTOFF_HZ > 0.0f, "Gyro cutoff must be positive");

// Keep the existing class name for sketch compatibility. Drift correction is
// deliberately the responsibility of the sensor calibration, not this filter.
class GyroDriftFilter {
public:
  GyroDriftFilter() { reset(); }

  void reset() {
    lp_[0] = lp_[1] = lp_[2] = 0.0f;
    seeded_ = false;
  }

  // Call only for a fresh sample; elapsedUs is time since the preceding fresh
  // sample, not the USB report period. All axes have identical bandwidth.
  // Invalid vectors are rejected together and cannot poison the filter state.
  bool process(float x, float y, float z, float &ox, float &oy, float &oz,
               uint32_t elapsedUs = 1000) {
    const bool accepted = isfinite(x) && isfinite(y) && isfinite(z) && elapsedUs != 0;
    if (accepted) {
      if (!seeded_) {
        lp_[0] = x;
        lp_[1] = y;
        lp_[2] = z;
        seeded_ = true;
      } else {
        // Exact first-order time constant, independent of sample cadence.
        const float alpha = 1.0f - expf(-6.28318530718f * GDF_CUTOFF_HZ *
                                        (static_cast<float>(elapsedUs) * 1.0e-6f));
        lp_[0] += alpha * (x - lp_[0]);
        lp_[1] += alpha * (y - lp_[1]);
        lp_[2] += alpha * (z - lp_[2]);
      }
    }
    ox = lp_[0];
    oy = lp_[1];
    oz = lp_[2];
    return accepted;
  }

private:
  float lp_[3];
  bool seeded_;
};

#endif
