#!/usr/bin/env bash
set -euxo pipefail

# If no version provided, extract from version.h
if [ "$#" -eq 0 ]; then
  SCRIPT_DIR="$(dirname "$0")"
  VERSION=$("${SCRIPT_DIR}/../scripts/get_version.sh")
  echo "Using version from version.h: ${VERSION}"
elif [ "$#" -eq 1 ]; then
  VERSION="$1"
else
  echo "Usage: $0 [version]" >&2
  exit 1
fi
ARCH=$(dpkg --print-architecture)
BUILD_DIR="build"
PKG_ROOT="$BUILD_DIR/pkg"
DEBIAN_DIR="$PKG_ROOT/DEBIAN"
BIN_DIR="$PKG_ROOT/usr/local/bin"

rm -rf "$BUILD_DIR"
mkdir -p "$DEBIAN_DIR" "$BIN_DIR"

if [ ! -f rmp-eval ]; then
  echo "rmp-eval binary not found. Build it first." >&2
  exit 1
fi

install -m 0755 rmp-eval "$BIN_DIR/rmp-eval"

cat <<CONTROL > "$DEBIAN_DIR/control"
Package: rmp-eval
Version: $VERSION
Section: utils
Priority: optional
Architecture: $ARCH
Maintainer: Micro Rokoku Maintainers <noreply@example.com>
Description: A simple utility to evaluate latencies on a Linux system and determine real-time capabilities.
CONTROL

dpkg-deb --build "$PKG_ROOT" "rmp-eval_${VERSION}_${ARCH}.deb"
