#include "gyro_drift_filter.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
void check(bool ok, const char *message) {
  if (!ok) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void near(float actual, float expected, float tolerance, const char *message) {
  check(std::isfinite(actual) && std::fabs(actual - expected) <= tolerance, message);
}

void slowRotationIsNotLearnedAsBias() {
  // 0.05 dps is smaller than the previous 0.244 dps soft deadzone, and a
  // 30-second steady rotation would previously have been learned as bias.
  GyroDriftFilter filter;
  float x, y, z;
  filter.process(0, 0, 0, x, y, z);
  const float rate = 0.05f * 16.384f;
  for (int i = 0; i < 30000; ++i) filter.process(rate, -rate, rate, x, y, z);
  near(x, rate, 1.0e-5f, "slow X motion survives prolonged rotation");
  near(y, -rate, 1.0e-5f, "slow negative Y motion survives prolonged rotation");
  near(z, rate, 1.0e-5f, "slow Z motion survives prolonged rotation");
  for (int i = 0; i < 100; ++i) filter.process(0, 0, 0, x, y, z);
  near(x, 0, 1.0e-6f, "stopping rotation leaves no learned opposite X drift");
  near(y, 0, 1.0e-6f, "stopping rotation leaves no learned opposite Y drift");
  near(z, 0, 1.0e-6f, "stopping rotation leaves no learned opposite Z drift");
}

void identicalAxisResponseAndLowLatency() {
  GyroDriftFilter filter;
  float x, y, z;
  filter.process(0, 0, 0, x, y, z);
  for (int i = 0; i < 5; ++i) {
    filter.process(100, -100, 100, x, y, z);
    near(x, -y, 1.0e-5f, "positive and negative axes have identical response");
    near(x, z, 1.0e-5f, "Z has the same bandwidth as X and Y");
    check(x > 0 && x <= 100, "step response does not overshoot");
  }
  check(x > 90, "step response reaches 90 percent within 5 ms");
  for (int i = 0; i < 5; ++i) filter.process(0, 0, 0, x, y, z);
  check(x < 10, "motion stop decays below 10 percent within 5 ms");
}

float stepAfterTenMs(uint32_t sampleUs) {
  GyroDriftFilter filter;
  float x, y, z;
  filter.process(0, 0, 0, x, y, z);
  for (uint32_t t = 0; t < 10000; t += sampleUs)
    filter.process(100, 0, 0, x, y, z, sampleUs);
  return x;
}

void timeConstantIndependentOfCadence() {
  const float at1kHz = stepAfterTenMs(1000);
  near(stepAfterTenMs(500), at1kHz, 2.0e-5f, "2 kHz samples preserve filter time constant");
  near(stepAfterTenMs(2000), at1kHz, 2.0e-5f, "500 Hz samples preserve filter time constant");
  GyroDriftFilter filter;
  float x, y, z;
  filter.process(0, 0, 0, x, y, z);
  const uint32_t jitterUs[] = {250, 750, 1600, 400, 2000, 1000, 3000, 1000};
  for (uint32_t dt : jitterUs) filter.process(100, 0, 0, x, y, z, dt);
  near(x, at1kHz, 2.0e-5f, "irregular fresh-sample timing preserves time constant");
}

void rejectsBadSamplesAndRecovers() {
  GyroDriftFilter filter;
  float x, y, z;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  check(!filter.process(nan, 1, 2, x, y, z), "reject NaN before first sample");
  near(x, 0, 0, "rejected first sample leaves zero output");
  check(filter.process(10, 20, 30, x, y, z), "accept finite first sample");
  near(x, 10, 0, "first valid sample seeds filter without startup ramp");
  check(!filter.process(40, inf, 60, x, y, z), "reject infinity in any axis");
  near(x, 10, 0, "invalid vector cannot partly update X");
  near(y, 20, 0, "invalid vector cannot poison Y state");
  near(z, 30, 0, "invalid vector cannot partly update Z");
  check(!filter.process(40, 50, 60, x, y, z, 0), "zero elapsed time does not double-filter");
  near(x, 10, 0, "duplicate timestamp holds output");
  check(filter.process(40, 50, 60, x, y, z), "recover after invalid input");
  check(x > 10 && x < 40 && std::isfinite(y) && std::isfinite(z), "recovery retains finite state");
  filter.reset();
  filter.process(-1, -2, -3, x, y, z);
  near(x, -1, 0, "reset reseeds after recalibration");
  near(y, -2, 0, "reset discards prior Y state");
  near(z, -3, 0, "reset discards prior Z state");
}

void attenuatesHighFrequencyNoise() {
  GyroDriftFilter filter;
  float x, y, z;
  filter.process(0, 0, 0, x, y, z);
  for (int i = 0; i < 1000; ++i) {
    const float noise = (i % 2) ? 100.0f : -100.0f;
    filter.process(noise, noise, noise, x, y, z);
  }
  check(std::fabs(x) < 30, "500 Hz alternating noise attenuated by more than 70 percent");
}
} // namespace

int main() {
  slowRotationIsNotLearnedAsBias();
  identicalAxisResponseAndLowLatency();
  timeConstantIndependentOfCadence();
  rejectsBadSamplesAndRecovers();
  attenuatesHighFrequencyNoise();
  std::cout << "PASS: 5 gyro filter scenarios\n";
}
