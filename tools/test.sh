#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
bash tests/redlink-protocol.sh
bash tests/redlink-radio.sh
bash tests/redlink-climate.sh
