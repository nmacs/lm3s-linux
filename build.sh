#!/bin/bash

ROOT=`pwd`

export PATH="$ROOT/../toolchain/arm-2011.03/bin:$PATH"

make clean
make -j 4 ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi- Image || exit 1

echo == Compiled ==
