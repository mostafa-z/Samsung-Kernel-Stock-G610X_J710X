#!/sbin/busybox sh

BB=/sbin/busybox

CACHE=$($BB blkid /dev/block/platform/13540000.dwmmc0/by-name/CACHE | $BB grep "f2fs" | $BB wc -l);

if [ ! -e /cache ]; then
	mkdir /cache;
	chown system:cache /cache;
	chmod 0770 /cache;
fi;

if [ "$CACHE" -eq "1" ]; then
	/system/bin/fsck.f2fs -a /dev/block/platform/13540000.dwmmc0/by-name/CACHE
	$BB mount -t f2fs -o nosuid,nodev /dev/block/platform/13540000.dwmmc0/by-name/CACHE /cache;
else
	$BB mount -t ext4 -o nosuid,nodev,noauto_da_alloc,errors=continue /dev/block/platform/13540000.dwmmc0/by-name/CACHE /cache;
fi;

if [ ! -e /cache/lost+found ]; then
	mkdir /cache/lost+found;
	chmod 0770 /cachelost+found;
fi;
