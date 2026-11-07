#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
shellcheck tests/*.sh tools/*.sh
cppcheck --enable=warning,performance,portability --error-exitcode=1 --inline-suppr --std=c++17 --suppress=missingIncludeSystem -I tests/redlink_fakes -I components/redlink components/redlink tests/*.cpp
