#!/bin/sh


ROOT=`pwd`
#TOOLCHAIN_BIN="$ROOT/../toolchain/usr/local/bin"
UBOOT_TOOLS_BIN="$ROOT/../u-boot-uwic/tools"
export PATH=$UBOOT_TOOLS_BIN:$PATH

image="$ROOT/uboot-linux.bin"
load_addr="0x60400000"

gzip --best --force --stdout "$ROOT/arch/arm/boot/Image" > "$ROOT/Image.gz"

mkimage -A arm -O linux -T kernel -C gzip -a 0x60008000 -e 0x60008000 -n "Linux kernel image" -d "$ROOT/Image.gz" "$ROOT/uboot-linux.bin" || exit 1

echo == Image ready ==

openocd -f "$ROOT/../u-boot-uwic/jtag/uwic.cfg" -c "load_sdram $image" || exit 1

echo == Image loaded ==
