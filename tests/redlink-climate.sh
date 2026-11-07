#!/usr/bin/env bash
# Offline actual-controller regressions: fake transport, no device/network/radio access.
# Run with: bash tests/redlink-climate.sh
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/redlink-climate.XXXXXX")"
trap 'rm -f "$binary"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I"$root/tests/redlink_fakes" -I"$root/components/redlink" \
  "$root/components/redlink/redlink.cpp" "$root/components/redlink/protocol.cpp" \
  "$root/tests/redlink_climate.cpp" -o "$binary"
"$binary"
