#!/bin/bash

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- exynos850_defconfig -j16 O=out
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- dtbs Image -j16 O=out

cp out/arch/arm64/boot/Image ~/uniLoader/blob/Image
cp out/arch/arm64/boot/dts/exynos/exynos850-a135f-4gb.dtb ~/uniLoader/blob/dtb

cd ~/uniLoader

~/uniLoader/make.sh
