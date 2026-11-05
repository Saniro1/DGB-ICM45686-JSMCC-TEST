#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/toolchain-env.sh"
if [[ ! -x "$ARDUINO_CLI" ]]; then
  echo 'Run scripts/setup-toolchain.sh first.' >&2
  exit 1
fi
output="${1:-$DGB_REPO_DIR/build}"
mkdir -p "$output"
output="$(cd -- "$output" && pwd)"
staging="$(mktemp -d "$output/source.XXXXXX")"
trap 'rm -rf -- "$staging"' EXIT
unzip -q "$DGB_REPO_DIR/DGB_Firmware_ICM45686_source.zip" -d "$staging"
patch --batch -d "$staging" -p1 --forward < "$DGB_REPO_DIR/patches/icm45686-driver-fix.patch"
cp "$DGB_REPO_DIR"/firmware-overrides/* "$staging/DGB_Firmware/"
# Limit peak memory while allowing independent compilation units to run together.
"$ARDUINO_CLI" compile \
  --fqbn rp2040:rp2040:waveshare_rp2040_zero:usbstack=tinyusb \
  --jobs "${DGB_BUILD_JOBS:-4}" \
  --warnings all \
  --output-dir "$output" \
  "$staging/DGB_Firmware"
python3 "$DGB_REPO_DIR/scripts/verify-uf2.py" "$output/DGB_Firmware.ino.uf2"
