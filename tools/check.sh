#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
prettier --check .
uvx ruff@0.14.3 check components
uvx ruff@0.14.3 format --check components
bash tools/format-cpp.sh --check
bash tools/static-checks.sh
