#!/usr/bin/env bash
set -euo pipefail

# Find version.h relative to this script's location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VERSION_H="${SCRIPT_DIR}/../include/version.h"

if [ ! -f "$VERSION_H" ]; then
  echo "Error: version.h not found at $VERSION_H" >&2
  exit 1
fi

# Extract version components
MAJOR=$(grep '#define VERSION_MAJOR' "$VERSION_H" | awk '{print $3}')
MINOR=$(grep '#define VERSION_MINOR' "$VERSION_H" | awk '{print $3}')
MICRO=$(grep '#define VERSION_MICRO' "$VERSION_H" | awk '{print $3}')

# Output format based on first argument
case "${1:-string}" in
  major)
    echo "$MAJOR"
    ;;
  minor)
    echo "$MINOR"
    ;;
  micro)
    echo "$MICRO"
    ;;
  string|*)
    echo "${MAJOR}.${MINOR}.${MICRO}"
    ;;
esac