#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
case "${1:---check}" in
  --write) flags=(-i) ;;
  --check) flags=(--dry-run --Werror) ;;
  *) echo "Usage: $0 [--check|--write]" >&2; exit 2 ;;
esac
find components tests -type f \( -name "*.cpp" -o -name "*.h" \) -print0 |
  xargs -0 uvx clang-format==18.1.8 "${flags[@]}"
