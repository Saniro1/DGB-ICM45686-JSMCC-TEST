#!/usr/bin/env bash
# Source this file to use the pinned, project-local Arduino toolchain.
DGB_REPO_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export DGB_TOOLCHAIN_DIR="${DGB_TOOLCHAIN_DIR:-$(dirname -- "$DGB_REPO_DIR")/toolchains}"
export ARDUINO_DIRECTORIES_DATA="$DGB_TOOLCHAIN_DIR/arduino-data"
export ARDUINO_DIRECTORIES_DOWNLOADS="$DGB_TOOLCHAIN_DIR/arduino-downloads"
export ARDUINO_DIRECTORIES_USER="$DGB_TOOLCHAIN_DIR/arduino-user"
# Arduino CLI requires its own proxy setting. Never disable TLS verification.
if [[ -n "${HTTPS_PROXY:-${https_proxy:-}}" ]]; then
  export ARDUINO_NETWORK_PROXY="${ARDUINO_NETWORK_PROXY:-${HTTPS_PROXY:-${https_proxy:-}}}"
fi
export ARDUINO_CLI="$DGB_TOOLCHAIN_DIR/arduino-cli-1.2.2/arduino-cli"
export PATH="$DGB_TOOLCHAIN_DIR/arduino-cli-1.2.2:$PATH"
