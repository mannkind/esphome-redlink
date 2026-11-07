#!/usr/bin/env bash
# Offline C++ behavior checks only: never loads an ESPHome config or opens a radio.
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
binary="$(mktemp "${TMPDIR:-/tmp}/redlink-protocol.XXXXXX")"
trap 'rm -f "$binary"' EXIT
"${CXX:-c++}" -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -I"$root/components/redlink" "$root/components/redlink/protocol.cpp" \
  "$root/tests/redlink_protocol.cpp" -o "$binary"
"$binary"
