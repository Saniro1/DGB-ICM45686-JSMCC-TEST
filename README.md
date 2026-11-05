# Dual ICM-45686 / DS4 test firmware

Adapted from Saniro1/DGB-ICM45686-JSM, branch `test/gyro-noise-filter`, commit `ab3f7d79e88744dcabbde9cb6c49075cfc9095f5`. Original history and source archive are retained. Maintained sources in `firmware-overrides` replace the extracted archive files during builds.

Target: Waveshare RP2040-Zero, two equally oriented ICM-45686 sensors. Wiring remains GP0 MISO, GP1 CS1, GP2 SCK, GP3 MOSI, GP5 CS2, GP9 touch send, GP12 touch receive, 3V3/GND.

## Corrections

- Per-sensor floating-point gyro bias measured only during an explicit stationary calibration window. Motion rejects the candidate; failed recalibration preserves prior offsets. Data-ready checks exclude repeated samples. Calibration is nonblocking.
- Symmetric 80 Hz gyro smoothing preserves slow rotations. The previous automatic bias learner could erase slow yaw and create opposite residual drift; it has been removed.
- Gyro and accelerometer share the same proper axis rotation `[x,-z,y]`, inherited from the latest gyro test. Both physical sensors must share mounting orientation. Confirm directions on your hardware.
- Accelerometer conversion now supplies 8192 counts/g rather than 2048. Gravity is preserved. All report axes saturate safely.
- DS4 timestamps now use 16/3 microsecond ticks rather than raw microseconds. The old encoding could make 1000 actual reports/s appear near 187.5 Hz to software deriving frequency from sensor timestamps.
- Fixed 1000 microsecond report scheduling, 1 ms USB interrupt endpoint, correct DS4 status/touch offsets and calibration feature reports. Successful submissions advance sequence counters.
- Touch measurements stop at their decision threshold instead of potentially spinning one million iterations. Serial diagnostics use bounded writes.

## Build and tests

Linux x86_64 prerequisites: GCC/G++, Python3, curl, tar, sha256sum, unzip, patch. Run:

```sh
scripts/test-host.sh
scripts/setup-toolchain.sh
scripts/build-firmware.sh
```

Pinned toolchain: Arduino CLI 1.2.2, Arduino-Pico 5.4.4, bundled TinyUSB. Network access needs GitHub release destinations and downloads.arduino.cc. Output is `build/DGB_Firmware.ino.uf2`; the build verifies its UF2 structure. Host tests exercise real driver code against simulated SPI; they do not establish physical sensor performance.

## Calibration and hardware acceptance

Place the device completely still before connecting it. Startup collects at least 1000 fresh samples per initialized sensor and at least one second of stable data. Initial gyro output stays neutral until calibration succeeds. Open the USB serial monitor at 115200 and send `?` for sensor/calibration status. Send `c` while completely stationary to recalibrate after warming up. Calibration times out after ten seconds; retry if it fails. Constant slow rotation about gravity cannot be reliably distinguished from zero-rate bias, so stationary placement is essential.

Startup calibration measures gyro offsets only. Actual accelerometer offset/gain calibration requires six measured +X/-X/+Y/-Y/+Z/-Z gravity poses; no fabricated coefficients are applied. In each pose verify approximately 1 g total magnitude and the expected signed axis. Rotate about each axis independently and confirm gyro direction, then check warm stationary drift for several minutes.

1000 Hz is the requested firmware/endpoint rate, not yet a measured hardware result. Verify actual USB arrivals independently of the corrected DS4 sensor clock, using a host HID capture or USB analyzer. Check both sensors, touch held/released, and serial connected/disconnected. Queue statistics from `?` describe firmware submissions, not USB bus arrival times.

## Validation status

Host filter, DS4 descriptor/report/timestamp, and IMU calibration/fusion tests pass. Full target compilation and UF2 structural verification also pass using a locally built Arduino ctags tool. The experimental UF2 and build details are in `test firmwares/`. Physical device testing, including drift and actual USB arrival rate, remains pending.
