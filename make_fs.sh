cd ./initramfs
find . | cpio -o -H newc > ../ramdisk.cpio

