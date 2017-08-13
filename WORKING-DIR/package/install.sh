#!/sbin/sh

if [ -e /system/xbin/su ]; then
	cp /system/xbin/su* /tmp/
fi;

dd if=/tmp/boot.img of=/dev/block/platform/13540000.dwmmc0/by-name/BOOT || exit 1

if [ -e /tmp/su ]; then
	cp /tmp/su* /system/xbin/
fi;

exit 0
