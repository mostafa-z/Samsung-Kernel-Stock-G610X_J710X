#!/sbin/busybox sh

# Kernel Tuning by Dorimanx, Mostafaz.

BB=/sbin/busybox

# protect init from oom
if [ -f /system/xbin/su ]; then
	su -c echo "-1000" > /proc/1/oom_score_adj;
fi;

OPEN_RW()
{
	if [ "$($BB mount | grep rootfs | cut -c 26-27 | grep -c ro)" -eq "1" ]; then
		$BB mount -o remount,rw /;
	fi;
	if [ "$($BB mount | grep system | grep -c ro)" -eq "1" ]; then
		$BB mount -o remount,rw /system;
	fi;
}
OPEN_RW;

# some nice thing for dev
if [ ! -e /cpufreq_l ]; then
	$BB ln -s /sys/devices/system/cpu/cpu0/cpufreq/ /cpufreq_l;
	$BB ln -sf /sys/devices/system/cpu/cpu4/cpufreq/ /cpufreq_b;
	$BB ln -s /sys/devices/system/cpu/cpufreq/mp-cpufreq/ /mp-cpufreq;
	$BB ln -s /sys/power/cpuhotplug/ /cpuhotplug;
fi;

# create init.d folder if missing
if [ ! -d /system/etc/init.d ]; then
	mkdir -p /system/etc/init.d/
	$BB chmod 755 /system/etc/init.d/;
fi;

OPEN_RW;

CRITICAL_PERM_FIX()
{
	# critical Permissions fix
	$BB chown -R root:root /tmp;
	$BB chown -R root:root /res;
	$BB chown -R root:root /sbin;
	$BB chmod -R 777 /tmp/;
	$BB chmod -R 775 /res/;
	$BB chmod -R 06755 /sbin/ext/;
	$BB chmod 06755 /sbin/busybox;
	$BB chmod 06755 /system/xbin/busybox;
}
CRITICAL_PERM_FIX;

SYSTEM_TUNING()
{
# KERNEL-TWEAKS
echo "0" > /proc/sys/vm/oom_kill_allocating_task;
echo "0" > /proc/sys/vm/panic_on_oom;
echo "30" > /proc/sys/kernel/panic;
echo "0" > /proc/sys/kernel/panic_on_oops;

# oom and mem perm fix
$BB chmod 666 /sys/module/lowmemorykiller/parameters/cost;
$BB chmod 666 /sys/module/lowmemorykiller/parameters/adj;
$BB chmod 666 /sys/module/lowmemorykiller/parameters/minfree

# Tune entropy parameters.
echo "512" > /proc/sys/kernel/random/read_wakeup_threshold; #default 64
echo "256" > /proc/sys/kernel/random/write_wakeup_threshold; #default 896

# take ownership and permissions
echo 0 > /cpuhotplug/enable;
sleep 0.5;

for i in cpu1 cpu2 cpu3 cpu4 cpu5 cpu6 cpu7; do
	$BB chown system /sys/devices/system/cpu/$i/online
	$BB chmod 666 /sys/devices/system/cpu/$i/online
done;

for i in cpu0 cpu4; do
$BB chown system /sys/devices/system/cpu/$i/cpufreq/*
$BB chown system /sys/devices/system/cpu/$i/cpufreq/*
$BB chmod 666 /sys/devices/system/cpu/$i/cpufreq/scaling_governor
$BB chmod 666 /sys/devices/system/cpu/$i/cpufreq/scaling_max_freq
$BB chmod 666 /sys/devices/system/cpu/$i/cpufreq/scaling_min_freq
$BB chmod 444 /sys/devices/system/cpu/$i/cpufreq/cpuinfo_cur_freq
$BB chmod 444 /sys/devices/system/cpu/$i/cpufreq/stats/*
done;

for i in 0 1; do
$BB chmod 666 /mp-cpufreq/cluster$i_max_freq
$BB chmod 666 /mp-cpufreq/cluster$i_min_freq
$BB chmod 444 /mp-cpufreq/cluster$i_freq_table
done;

#Governor Tuning
C0=cpufreq_l
C1=cpufreq_b
G0=interactive
G1=cultivation

for i in $C0 $C1; do
	for j in $G0 $G1; do
		echo $j > /$i/scaling_governor;
		sleep 0.5;
		#defaults : interactive ,cultivation
		echo 99 > /$i/$j/go_hispeed_load; #default 89 ,99
		echo 902000 > /$i/$j/hispeed_freq; #default 900000 ,max_freq
		echo 40000 > /$i/$j/min_sample_time; #default 40000 ,80000
		echo "70 546000:65 676000:70 757000:75 839000:80 902000:85 1014000:90 1144000:95 1248000:100 1352000:110 1482000:170 1586000:200" > /$i/$j/target_loads; #default 75 1248000:85 ,90
		echo 20000 > /$i/$j/timer_rate; #default 20000
		echo 80000 > /$i/$j/timer_slack; #default 20000 ,80000
		echo 50000 > /$i/$j/timer_rate_screenoff; #default x ,50000
		echo 0 > /$i/$j/fastlane; #default x ,0
		echo 0 > /$i/$j/above_hispeed_delay; #default x ,x
		echo 1 > /$i/$j/align_windows; #default x ,0
		echo 1 > /$i/$j/io_is_busy; #default 0 ,0
		echo 100000 > /$i/$j/max_freq_hysteresis; #default x ,0
		$BB sync;
	done;
done;

#both cluster have cultivation
echo $G1 > /$C0/scaling_governor;
echo $G1 > /$C1/scaling_governor;

echo 1 > /cpuhotplug/enable;
}

if [ ! -d /data/.gabriel ]; then
	$BB mkdir -p /data/.gabriel;
fi;

if [ ! -d /data/.gabriel/logs ]; then
	$BB mkdir -p /data/.gabriel/logs;
fi;

OPEN_RW;

# set system tuning.
SYSTEM_TUNING;

# Start any init.d scripts that may be present in the rom or added by the user
$BB chmod -R 755 /system/etc/init.d/;
if [ "$init_d" == "on" ]; then
	(
		$BB nohup $BB run-parts /system/etc/init.d/ > /data/.gabriel/init.d.txt &
	)&
else
	if [ -e /system/etc/init.d/99SuperSUDaemon ]; then
		$BB nohup $BB sh /system/etc/init.d/99SuperSUDaemon > /data/.gabriel/root.txt &
	else
		echo "no root script in init.d";
	fi;
fi;

OPEN_RW;

# Fix critical perms again after init.d mess
CRITICAL_PERM_FIX;

# disable block iostats
for i in /sys/block/*/queue; do
	echo 0 > $i/iostats
done;

# Restore selinux
echo "1" > /sys/fs/selinux/enforce

# trim partitions on boot
fstrim -v /system
fstrim -v /cache
fstrim -v /data

TIME_NOW=$(date)
echo "$TIME_NOW" > /data/.gabriel/boot.txt
