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

$BB chmod 666 /mp-cpufreq/cluster0_max_freq
$BB chmod 666 /mp-cpufreq/cluster0_min_freq
$BB chmod 444 /mp-cpufreq/cluster0_freq_table
$BB chmod 666 /mp-cpufreq/cluster1_max_freq
$BB chmod 666 /mp-cpufreq/cluster1_min_freq
$BB chmod 444 /mp-cpufreq/cluster1_freq_table

#Governor Tuning
C0=cpufreq_l
C1=cpufreq_b
G0=cultivation
G1=interactive

		#defaults : interactive ,cultivation
		echo $G0 > /$C0/scaling_governor;
		echo $G1 > /$C1/scaling_governor;
		sleep 0.5;

		# Little cluster
		echo "45 546000:50 676000:55 757000:60 839000:65 902000:70 1014000:75 1144000:80 1248000:85 1352000:90 1482000:95 1586000:100" > /$C0/$G0/target_loads; #default 75 1248000:85 ,90
		echo 20000 > /$C0/$G0/timer_rate; #default 20000
		echo 0 > /$C0/$G0/above_hispeed_delay; #default x ,x
		echo 100 > /$C0/$G0/go_hispeed_load; #default 89 ,99
		echo 0 > /$C0/$G0/hispeed_freq; #default 900000 ,max_freq
		echo 60000 > /$C0/$G0/min_sample_time; #default 40000 ,80000
		echo 80000 > /$C0/$G0/max_freq_hysteresis; #default x ,0
		echo 30000 > /$C0/$G0/timer_slack; #default 20000 ,80000
		echo 50000 > /$C0/$G0/timer_rate_screenoff; #default x ,50000
		echo 0 > /$C0/$G0/fastlane; #default x ,0
		echo 0 > /$C0/$G0/align_windows; #default x ,0
		echo 0 > /$C0/$G0/io_is_busy; #default 0 ,0

		# big cluster
		echo "90 1248000:95" > /$C1/$G1/target_loads; #default 75 1248000:85 ,90
		echo 30000 > /$C1/$G1/timer_rate; #default 20000
		echo "20000 902000:40000 1248000:20000" > /$C1/$G1/above_hispeed_delay; #default x ,x
		echo 90 > /$C1/$G1/go_hispeed_load; #default 89 ,99
		echo 902000 > /$C1/$G1/hispeed_freq; #default 900000 ,max_freq
		echo 20000 > /$C1/$G1/min_sample_time; #default 40000 ,80000
		echo 80000 > /$C1/$G1/max_freq_hysteresis; #default x ,0
		echo 30000 > /$C1/$G1/timer_slack; #default 20000 ,80000
		echo 50000 > /$C1/$G1/timer_rate_screenoff; #default x ,50000
		echo 1 > /$C1/$G1/fastlane; #default x ,0
		echo 0 > /$C1/$G1/align_windows; #default x ,0
		echo 0 > /$C1/$G1/io_is_busy; #default 0 ,0

echo 1 > /cpuhotplug/enable;
}

VM_DEFAULT()
{
	echo 90 > /proc/sys/vm/dirty_ratio; #default 0
	echo 70 > /proc/sys/vm/dirty_background_ratio; #default 0
	echo 200 > /proc/sys/vm/dirty_expire_centisecs; #default 200
	echo 500 > /proc/sys/vm/dirty_writeback_centisecs; #default 500
	echo 60 > /proc/sys/vm/swappiness; #default 100
	echo 10 > /proc/sys/vm/vfs_cache_pressure; #default 100
	echo 8192 > /proc/sys/vm/min_free_kbytes; #default 6806
	echo "256 32" > /proc/sys/vm/lowmem_reserve_ratio; #default 256 32
	echo 3 > /proc/sys/vm/drop_caches;
	$BB sleep 0.5s
	$BB sync
}

VM_SPEEDMODE()
{
	echo 5 > /proc/sys/vm/dirty_background_ratio
	echo 200 > /proc/sys/vm/dirty_expire_centisecs
	echo 20 > /proc/sys/vm/dirty_ratio
	echo 1500 > /proc/sys/vm/dirty_writeback_centisecs
	echo 12288 > /proc/sys/vm/min_free_kbytes
	echo 0 > /proc/sys/vm/swappiness
	echo 100 > /proc/sys/vm/vfs_cache_pressure
	echo 0 > /proc/sys/vm/drop_caches
	$BB sleep 0.5s
	$BB sync
}

VM_MITTIADJ()
{
	echo 10 > /proc/sys/vm/dirty_background_ratio
	echo 500 > /proc/sys/vm/dirty_expire_centisecs
	echo 10 > /proc/sys/vm/dirty_ratio
	echo 100 > /proc/sys/vm/dirty_writeback_centisecs
	echo 8192 > /proc/sys/vm/min_free_kbytes
	echo 70 > /proc/sys/vm/swappiness
	echo 500 > /proc/sys/vm/vfs_cache_pressure
	echo 0 > /proc/sys/vm/drop_caches
	$BB sleep 0.5s
	$BB sync
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
VM_DEFAULT;

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
