# Calibration / DS4 1 kHz test build

Target: Waveshare RP2040-Zero with two equally oriented ICM-45686 sensors and the wiring documented in the repository README. This is experimental firmware, compiled successfully but not tested on physical hardware. Requested 1000 Hz USB reporting is not a measured result.

## Flash

1. Keep a copy of your previous working UF2.
2. Hold BOOT while connecting the RP2040 by USB, then release it.
3. Copy `DGB_ICM45686_calibration_DS4_1kHz_TEST.uf2` to the RPI-RP2 drive.
4. Let the device reboot. Leave it completely still for calibration.
5. Open the serial monitor at 115200 and send `?`. Confirm calibration is complete and both IMUs are OK. If calibration fails, leave the device still and send `c` to retry. In v1, gyro output remains neutral until initial calibration succeeds; v2 removes this lockout.

Check each rotation axis, stationary drift after warmup, gravity magnitude (approximately 1 g), touch operation and actual host report arrivals. Startup calibration corrects gyro bias; accelerometer six-face offset/gain calibration remains a hardware measurement task.

## Build evidence

- Firmware source: commit 3c7c941, unchanged apart from build-script support for selecting a local ctags executable.
- Arduino CLI 1.2.2; Arduino-Pico 5.4.4; bundled TinyUSB 3.7.2.
- Arduino ctags source abc8fca7499f44c725122881cd380a88c37abe0e built locally with its internal `__unused__` macro renamed to avoid a modern glibc macro conflict. Selected using DGB_CTAGS_DIR and the Arduino runtime.tools.ctags.path build property.
- Compile passed: 85,092 bytes program, 14,428 bytes globals.
- UF2 verification passed: 400 RP2040 blocks, 204,800 bytes.
- SHA256: 81479883685fb5352144d7b59f380d761109cd2aec0d29f1cf5ffd3417404c7c
- Host gyro, DS4 and IMU regression tests passed.
