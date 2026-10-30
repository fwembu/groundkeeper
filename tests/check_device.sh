#!/bin/sh
# SPDX-License-Identifier: GPL-3.0-only
# Groundkeeper: quick checks of the real FM-1 build, without the JieLi toolchain.
#   1. a syntax check of the device build with the host's clang (the whole unity build: platform.c, libc.c,
#      usb.c ... are included, unlike in host mode), so name clashes and typos show up here, not in CI;
#   2. tools/build.py's source checks (no register addresses, volatile casts or asm outside hal/).
# What it can't check: the ROM-address and size checks need the real compiler (CI does those).
set -e
cd "$(dirname "$0")/.."
for B in 0 1; do
  clang -m32 -fsyntax-only -ffreestanding -fno-builtin -Wall -Wno-unused-function -DBOARD_SMK37=$B \
    -DAPP_VER='"dev"' -DFELUCCA_ID='"FM-1_990"' -Ifirmware/hal -Ifirmware/src -Ifirmware/gen firmware/src/dinghy.c
done
python3 -I -c "import sys; sys.path.insert(0, 'tools'); import build; e = build.mmio_check(); print('\n'.join(e)); sys.exit(1 if e else 0)"
echo "device checks: ok"
