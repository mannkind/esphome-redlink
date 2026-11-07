#!/usr/bin/env bash
# Offline actual-driver regressions: no ESPHome config, device, network, or radio access.
# Run with: bash tests/redlink-radio.sh
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/redlink-radio.XXXXXX")"
trap 'rm -f "$binary"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I"$root/tests/redlink_fakes" -I"$root/components/redlink" \
  "$root/components/redlink/radio.cpp" "$root/components/redlink/protocol.cpp" \
  "$root/tests/redlink_radio.cpp" -o "$binary"
"$binary"
