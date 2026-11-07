#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
prettier --write .
uvx ruff@0.14.3 format components
bash tools/format-cpp.sh --write
