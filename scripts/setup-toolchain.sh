#!/usr/bin/env bash
set -euo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/toolchain-env.sh"
for tool in curl tar sha256sum python3 unzip patch; do
  command -v "$tool" >/dev/null || { echo "Missing prerequisite: $tool" >&2; exit 1; }
done
if [[ "$(uname -s)-$(uname -m)" != Linux-x86_64 ]]; then
  echo 'This installer supports Linux x86_64; use Arduino CLI 1.2.2 and Arduino-Pico 5.4.4 on other hosts.' >&2
  exit 1
fi
mkdir -p "$(dirname -- "$ARDUINO_CLI")" "$ARDUINO_DIRECTORIES_DATA" "$ARDUINO_DIRECTORIES_DOWNLOADS" "$ARDUINO_DIRECTORIES_USER"
if [[ ! -x "$ARDUINO_CLI" ]]; then
  archive="$(mktemp "$DGB_TOOLCHAIN_DIR/arduino-cli-1.2.2/download.XXXXXX")"
  trap 'rm -f -- "$archive"' EXIT
  curl --fail --location --retry 2 \
    https://github.com/arduino/arduino-cli/releases/download/v1.2.2/arduino-cli_1.2.2_Linux_64bit.tar.gz \
    --output "$archive"
  # Published v1.2.2 checksum: https://github.com/arduino/arduino-cli/releases/download/v1.2.2/1.2.2-checksums.txt
  printf '%s  %s\n' a5ad085d1b02fd4010df18b1bbf20453276efe6c6e2ac323baa0cac4e96a85b4 "$archive" | sha256sum --check --status
  tar -xzf "$archive" -C "$(dirname -- "$ARDUINO_CLI")" arduino-cli
fi
"$ARDUINO_CLI" version
index=https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
# Both downloads.arduino.cc and GitHub release downloads must be allowed.
"$ARDUINO_CLI" core update-index --additional-urls "$index"
"$ARDUINO_CLI" core install rp2040:rp2040@5.4.4 --additional-urls "$index"
"$ARDUINO_CLI" core list
