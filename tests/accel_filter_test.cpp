#include "gyro_drift_filter.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  GyroDriftFilter filter(40.0f);
  float x, y, z, previous = 0, maximumStep = 0;
  filter.process(0, 0, 1, x, y, z, 625);
  // Two sensors alternating by 0.1 g: raw steps exceed SDL's 0.075 g gate.
  for (int n = 0; n < 5000; ++n) {
    filter.process(n % 2 ? 0.05f : -0.05f, 0, 1, x, y, z, 625);
    maximumStep = fmaxf(maximumStep, fabsf(x - previous));
    previous = x;
    assert(fabsf(z - 1) < 0.00001f); // Gravity is not subtracted.
  }
  assert(maximumStep < 0.075f);
  for (int n = 0; n < 1000; ++n) filter.process(0.5f, -0.5f, 0.70710678f, x, y, z, 625);
  assert(fabsf(x - 0.5f) < 0.00001f && fabsf(y + 0.5f) < 0.00001f);
  assert(fabsf(z - 0.70710678f) < 0.00001f);
  std::cout << "PASS: accel noise attenuation, gravity preservation, sustained tilt\n";
}
