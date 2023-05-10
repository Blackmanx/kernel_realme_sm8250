#!/bin/bash

#
# Clone proton-clang toolchain if needed
#

KERNEL_DEFCONFIG=vendor/sm8250_defconfig
DIR=$PWD
export ARCH=arm64
export SUBARCH=arm64
export CLANG_PATH="~/toolchains/bin"
export PATH="$CLANG_PATH:$PATH"
export CROSS_COMPILE=aarch64-linux-gnu-
export CROSS_COMPILE_ARM32=arm-linux-gnueabi-
export KBUILD_BUILD_USER=Amog
export KBUILD_BUILD_HOST=Us

echo
echo "Kernel is going to be built using $KERNEL_DEFCONFIG"
echo

make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip LLVM=1 LLVM_IAS=1 O=out $KERNEL_DEFCONFIG

make CC=clang AR=llvm-ar NM=llvm-nm OBJCOPY=llvm-objcopy OBJDUMP=llvm-objdump STRIP=llvm-strip LLVM=1 LLVM_IAS=1 O=out -j$(nproc --all)
