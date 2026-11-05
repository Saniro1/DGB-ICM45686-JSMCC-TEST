#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT
unzip -q DGB_Firmware_ICM45686_source.zip -d "$stage"
patch --batch -d "$stage" -p1 < patches/icm45686-driver-fix.patch >/dev/null
g++ -std=c++17 -Wall -Wextra -Werror -Ifirmware-overrides tests/gyro_filter_test.cpp -o "$stage/gyro"
g++ -std=c++17 -Wall -Wextra -Werror -Itests/ds4_stubs -Ifirmware-overrides tests/test_ds4.cpp firmware-overrides/usb_ds4_hid.cpp -o "$stage/ds4"
g++ -std=c++17 -Wall -Wextra -Werror -Itests/imu_stubs -Ifirmware-overrides -I"$stage/DGB_Firmware" tests/test_imu_driver.cpp firmware-overrides/icm45686_spi.cpp -o "$stage/imu"
"$stage/gyro"
"$stage/ds4"
"$stage/imu"
