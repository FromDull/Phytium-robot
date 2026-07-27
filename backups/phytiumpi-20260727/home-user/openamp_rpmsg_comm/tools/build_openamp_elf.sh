#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PHYTIUM_PI_OS_DIR="${PHYTIUM_PI_OS_DIR:-$(cd "$ROOT/../phytium-pi-os" && pwd)}"
STANDALONE_DIR="$PHYTIUM_PI_OS_DIR/output/build/phytium-standalone-openamp-v1.0"
APP_DIR="$STANDALONE_DIR/example/system/amp/openamp_for_linux"
TARGET_SRC="$APP_DIR/src"
HOST_DIR="${HOST_DIR:-$PHYTIUM_PI_OS_DIR/output/host}"
ELF="$APP_DIR/phytiumpi_aarch64_firefly_openamp_core0.elf"

if [[ ! -d "$TARGET_SRC" || ! -f "$HOST_DIR/etc/profile.d/phytium_dev.sh" ]]; then
    echo "Phytium standalone build tree is not ready: $APP_DIR" >&2
    exit 1
fi

cp "$ROOT/src/rpmsg_protocol.c" "$ROOT/src/rpmsg_protocol.h" "$TARGET_SRC/"
cp "$ROOT"/remote_firmware/*.[ch] "$TARGET_SRC/"

verify_sources() {
    cmp "$ROOT/src/rpmsg_protocol.c" "$TARGET_SRC/rpmsg_protocol.c"
    cmp "$ROOT/src/rpmsg_protocol.h" "$TARGET_SRC/rpmsg_protocol.h"
    for source in "$ROOT"/remote_firmware/*.[ch]; do
        cmp "$source" "$TARGET_SRC/$(basename "$source")"
    done
}

verify_sources
export HOST_DIR
# shellcheck disable=SC1090
source "$HOST_DIR/etc/profile.d/phytium_dev.sh"
make -C "$APP_DIR" all -j"${JOBS:-4}"
verify_sources

if [[ ! -s "$ELF" ]]; then
    echo "OpenAMP ELF was not generated: $ELF" >&2
    exit 1
fi

echo "OpenAMP ELF ready: $ELF"
