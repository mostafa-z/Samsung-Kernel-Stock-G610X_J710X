#!/sbin/busybox sh

#Credits:
# Zacharias.maladroit
# Voku1987
# Collin_ph@xda
# Dorimanx@xda
# Gokhanmoral@xda
# Johnbeetee
# Alucard_24@xda
# Mostafaz@xda

# TAKE NOTE THAT LINES PRECEDED BY A "#" IS COMMENTED OUT.
#
# This script must be activated after init start =< 25sec or parameters from /sys/* will not be loaded.

BB=/sbin/busybox

# change mode for /tmp/
ROOTFS_MOUNT=$(mount | grep rootfs | cut -c26-27 | grep -c rw)
if [ "$ROOTFS_MOUNT" -eq "0" ]; then
	mount -o remount,rw /;
fi;
chmod -R 777 /tmp/;

# ==============================================================
# GLOBAL VARIABLES || without "local" also a variable in a function is global
# ==============================================================

FILE_NAME=$0;
DATA_DIR=/data/.gabriel;

# ==============================================================
# INITIATE
# ==============================================================

# For CHARGER CHECK.
echo "1" > /data/gabriel_cortex_sleep;

rm -f /cache/uksm_state
rm -f /cache/max_cpu_percentage
rm -f /cache/power_efficient
rm -f /cache/hmp_up_threshold
rm -f /cache/hmp_down_threshold

# ==============================================================
# KERNEL-TWEAKS
# ==============================================================
KERNEL_TWEAKS()
{
		echo "0" > /proc/sys/vm/oom_kill_allocating_task;
		echo "0" > /proc/sys/vm/panic_on_oom;
		echo "30" > /proc/sys/kernel/panic;
		echo "0" > /proc/sys/kernel/panic_on_oops;
}
KERNEL_TWEAKS;

# ==============================================================
# TWEAKS: if Screen-ON
# ==============================================================
AWAKE_MODE()
{
if [ "$(cat /data/gabriel_cortex_sleep)" -eq "1" ]; then
	echo "546000" > /mp-cpufreq/cluster0_min_freq;
	echo "1586000" > /mp-cpufreq/cluster0_max_freq;
	echo "546000" > /mp-cpufreq/cluster1_min_freq;
	echo "1586000" > /mp-cpufreq/cluster1_max_freq;

	echo "$(cat /cache/uksm_state)" > /sys/kernel/mm/uksm/run;
	echo "$(cat /cache/max_cpu_percentage)" > /sys/kernel/mm/uksm/max_cpu_percentage;

	echo "$(cat /cache/power_efficient)" > /sys/module/workqueue/parameters/power_efficient;

	echo "$(cat /cache/hmp_up_threshold)" > /sys/kernel/hmp/up_threshold;
	echo "$(cat /cache/hmp_down_threshold)" > /sys/kernel/hmp/down_threshold;

	echo "1" > /sys/kernel/printk_mode/printk_mode;

	echo "0" > /data/gabriel_cortex_sleep
fi
}

# ==============================================================
# TWEAKS: if Screen-OFF
# ==============================================================
SLEEP_MODE()
{
	CHARGER_STATE=$(cat /sys/devices/battery/power_supply/battery/charge_now)

if [ "$CHARGER_STATE" -eq "0" ]; then
	echo "546000" > /mp-cpufreq/cluster0_min_freq;
	echo "1352000" > /mp-cpufreq/cluster0_max_freq;
	echo "546000" > /mp-cpufreq/cluster1_min_freq;
	echo "902000" > /mp-cpufreq/cluster1_max_freq;

	echo "$(cat /sys/kernel/mm/uksm/run)" > /cache/uksm_state;
	echo "1" > /sys/kernel/mm/uksm/run;
	echo "$(cat /sys/kernel/mm/uksm/max_cpu_percentage)" > /cache/max_cpu_percentage;
	echo "5" > /sys/kernel/mm/uksm/max_cpu_percentage;

	echo "$(cat /sys/module/workqueue/parameters/power_efficient)" > /cache/power_efficient;
	echo "1" > /sys/module/workqueue/parameters/power_efficient;

	echo "$(cat /sys/kernel/hmp/up_threshold)" > /cache/hmp_up_threshold;
	echo "1000" > /sys/kernel/hmp/up_threshold;
	echo "$(cat /sys/kernel/hmp/down_threshold)" > /cache/hmp_down_threshold;
	echo "500" > /sys/kernel/hmp/down_threshold;

	echo "0" > /sys/kernel/printk_mode/printk_mode;

	echo "1" > /data/gabriel_cortex_sleep
fi
}

# ==============================================================
# Background process to check screen state
# ==============================================================

while :
	do
	while [ "$(cat /sys/module/decon/parameters/sleep_state)" -ne "0" ]; do
		sleep "3"
	done
	# AWAKE State. all system ON
	AWAKE_MODE;

	while [ "$(cat /sys/module/decon/parameters/sleep_state)" -ne "1" ]; do
		sleep "3"
	done
	# SLEEP state. All system to power save
	SLEEP_MODE;
done
