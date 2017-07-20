#!/tmp/busybox sh
# Detects filesystem type automatically and mounts it
# Nedded because now we can have f2fs filesystem
# Credits for that goes to Dorimanx and Blastagator
# This workaround was done by Computoncio

BB=/tmp/busybox

systemfs=$(eval $($BB blkid /dev/block/platform/13540000.dwmmc0/by-name/SYSTEM | $BB awk ' { print $3 } '); $BB echo $TYPE);
datafs=$(eval $($BB blkid /dev/block/platform/13540000.dwmmc0/by-name/USERDATA | $BB awk ' { print $3 } '); $BB echo $TYPE);
DATA_MOUNT=$($BB mount | $BB grep /data | $BB wc -l);

$BB mount -t $systemfs /dev/block/platform/13540000.dwmmc0/by-name/SYSTEM /system;

if [ "$DATA_MOUNT" -eq "0" ]; then
	$BB mount -t $datafs /dev/block/platform/13540000.dwmmc0/by-name/USERDATA /data;
fi;

