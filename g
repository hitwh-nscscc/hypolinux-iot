#!/bin/sh

echo Make menu:
echo 1: Make menuconfig and build kernel
echo 2: Only build kernel image
echo 3: Build the kernel modiules
echo 4: Use embad compiler
echo 5: objdump

echo
echo -n "Please input your choice: "
read INPUT

if [ $INPUT -eq 1 ]
then
		make menuconfig
		make CROSS_COMPILE=/home/liuqi/toolchain/gcc-cross-4.4/install/bin/mips64el-linux- ARCH=mips -j 8
elif [ $INPUT -eq 2 ]
then
		make CROSS_COMPILE=/home/liuqi/toolchain/gcc-cross-4.4/install/bin/mips64el-linux- ARCH=mips -j 8
elif [ $INPUT -eq 3 ]
then
		make modules_install INSTALL_MOD_PATH=./lib_modules/ ARCH=mips CROSS_COMPILE=/home/liuqi/toolchain/gcc-cross-4.4/install/bin/mips64el-linux- -j 8
elif [ $INPUT -eq 4 ]
then
		make menuconfig ARCH=mips
		make ARCH=mips CROSS_COMPILE=/opt/gcc-4.3-ls232/bin/mipsel-linux- -j 4 && cp -av vmlinux /tftp
elif [ $INPUT -eq 5 ]
then
		/home/lichao/opt/gcc-4.3-ls32/bin/mipsel-linux-objdump -d vmlinux > 222.S
else
echo Error input choice!!
fi
