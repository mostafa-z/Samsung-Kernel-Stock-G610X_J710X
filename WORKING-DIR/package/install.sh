#!/sbin/sh

dd if=/tmp/boot.img of=/dev/block/platform/13540000.dwmmc0/by-name/BOOT || exit 1
exit 0
