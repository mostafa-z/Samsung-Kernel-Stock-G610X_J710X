/*
 * drivers/cpufreq/cpufreq_gabriel_plus.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Copyright (c) 2017, Mostafaz <mostafazarghami@gmail.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Mike Chan (mike@android.com)
 *
 * - Changelog:
 *
 * 'gabriel_plus' governor is based on the 'interactive'.
 * 0.1 : initial version adapted from samsung interactive governor + align windows.
 * 0.2 : add max_freq_hysteresis
 * 0.3 : add timer_rate_idle
 * 0.4 : add idle_threshold tunable
 * 0.5 : add frequency calculation threshold
 * 0.6 : add frequency responsiveness threshold
 * 0.7 : add timer_rate_idle frequency threshold
 * 0.8 : add ramp up threshold if cpu_load exceeds
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/ipa.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <linux/pm_qos.h>

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
#include <soc/samsung/cpufreq.h>
#endif

#if defined(CONFIG_CPU_THERMAL_IPA) || defined(CONFIG_EXYNOS_HOTPLUG_GOVERNOR)
#include "cpu_load_metric.h"
#endif

#define TASK_NAME_LEN 15
#define DEFAULT_TARGET_LOAD 80
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
#define DEFAULT_ABOVE_HISPEED_DELAY DEFAULT_TIMER_RATE
#define DEFAULT_GO_HISPEED_LOAD 89
#define DEFAULT_HISPEED_FREQ 902000
#define DEFAULT_MIN_SAMPLE_TIME 0
#define DEFAULT_TIMER_SLACK (1 * DEFAULT_TIMER_RATE)

#define DEFAULT_TIMER_RATE_IDLE (60 * USEC_PER_MSEC)
#define DEFAULT_TIMER_RATE_IDLE_FREQ 902000
#define DEFAULT_IDLE_THRESHOLD 20
#define DEFAULT_BUMP_FREQ_WEIGHT 150
#define DEFAULT_MAX_LOCAL_LOAD 100

#define FREQ_RESPONSIVENESS			1248000

#define PUMP_INC_STEP_AT_MIN_FREQ	6
#define PUMP_INC_STEP				3
#define PUMP_DEC_STEP_AT_MIN_FREQ	3
#define PUMP_DEC_STEP				1

struct cpufreq_gabriel_plus_cpuinfo {
	struct timer_list cpu_timer;
	struct timer_list cpu_slack_timer;
	spinlock_t load_lock; /* protects the next 4 fields */
	u64 time_in_idle;
	u64 time_in_idle_timestamp;
	u64 cputime_speedadj;
	u64 cputime_speedadj_timestamp;
#ifdef CONFIG_LOAD_BASED_CORE_CURRENT_CAL
	unsigned int pre_cpu_for_load;
	u64 curr_speed_total_time;
	u64 curr_speed_idle_time;
#endif
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	spinlock_t target_freq_lock; /*protects target freq */
	unsigned int target_freq;
	unsigned int floor_freq;
	u64 pol_floor_val_time; /* policy floor_validate_time */
	u64 loc_floor_val_time; /* per-cpu floor_validate_time */
	u64 pol_hispeed_val_time; /* policy hispeed_validate_time */
	u64 loc_hispeed_val_time; /* per-cpu hispeed_validate_time */
	u64 max_freq_hyst_start_time;
	struct rw_semaphore enable_sem;
	int governor_enabled;
	/* record the load of last 5 sampling intervals*/
	unsigned int prev_load[5];
	unsigned int prev_load_idx;
};

static DEFINE_PER_CPU(struct cpufreq_gabriel_plus_cpuinfo, cpuinfo);

struct task_struct *speedchp_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;
static struct mutex gov_lock;

static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};
static unsigned int default_above_hispeed_delay[] = {
	DEFAULT_ABOVE_HISPEED_DELAY };

struct cpufreq_gabriel_plus_tunables {
	int usage_count;
	unsigned int hispeed_freq;
	unsigned int freq_calc_thresh;
	unsigned long go_hispeed_load;
	spinlock_t target_loads_lock;
	unsigned int *target_loads;
	int ntarget_loads;
	unsigned long min_sample_time;
	unsigned long timer_rate;
	unsigned long prev_timer_rate;
	unsigned long timer_rate_idle;
	unsigned long timer_rate_idle_freq;
	unsigned long idle_threshold;
	unsigned long max_local_load;
	unsigned long bump_freq_weight;
	spinlock_t above_hispeed_delay_lock;
	unsigned int *above_hispeed_delay;
	int nabove_hispeed_delay;
	int boost_val;
	int boostpulse_duration_val;
	u64 boostpulse_endtime;
	bool boosted;
	int timer_slack_val;
	bool io_is_busy;
	unsigned int max_freq_hysteresis;
	bool align_windows;
	unsigned int *policy;
	/*
	 * CPUs frequency scaling
	 */
	int freq_responsiveness;
	int pump_inc_step;
	int pump_inc_step_at_min_freq;
	int pump_dec_step;
	int pump_dec_step_at_min_freq;
};

/* For cases where we have single governor instance for system */
static struct cpufreq_gabriel_plus_tunables *common_tunables;
static struct cpufreq_gabriel_plus_tunables *tuned_parameters[NR_CPUS] = {NULL, };

static struct attribute_group *get_sysfs_attr(void);

/* Round to starting jiffy of next evaluation window */
static u64 round_to_nw_start(u64 jif,
			     struct cpufreq_gabriel_plus_tunables *tunables)
{
	unsigned long step = usecs_to_jiffies(tunables->timer_rate);
	u64 ret;

	if (tunables->align_windows) {
		do_div(jif, step);
		ret = (jif + 1) * step;
	} else {
		ret = jiffies + usecs_to_jiffies(tunables->timer_rate);
	}

	return ret;
}

static void cpufreq_gabriel_plus_timer_resched(
	struct cpufreq_gabriel_plus_cpuinfo *pcpu)
{
	struct cpufreq_gabriel_plus_tunables *tunables =
		pcpu->policy->governor_data;
	unsigned long expires;
	unsigned long flags;
	u64 now = ktime_to_us(ktime_get());

	spin_lock_irqsave(&pcpu->load_lock, flags);
	pcpu->time_in_idle =
		get_cpu_idle_time(smp_processor_id(),
				  &pcpu->time_in_idle_timestamp,
				  tunables->io_is_busy);
	pcpu->cputime_speedadj = 0;
	pcpu->cputime_speedadj_timestamp = pcpu->time_in_idle_timestamp;
	expires = jiffies + usecs_to_jiffies(tunables->timer_rate);
	mod_timer_pinned(&pcpu->cpu_timer, expires);

	if (tunables->timer_slack_val >= 0 &&
	    (pcpu->target_freq > pcpu->policy->min ||
		(pcpu->target_freq == pcpu->policy->min &&
		 now < tunables->boostpulse_endtime))) {
		expires += usecs_to_jiffies(tunables->timer_slack_val);
		mod_timer_pinned(&pcpu->cpu_slack_timer, expires);
	}

	spin_unlock_irqrestore(&pcpu->load_lock, flags);
}

/* The caller shall take enable_sem write semaphore to avoid any timer race.
 * The cpu_timer and cpu_slack_timer must be deactivated when calling this
 * function.
 */
static void cpufreq_gabriel_plus_timer_start(
	struct cpufreq_gabriel_plus_tunables *tunables, int cpu)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	unsigned long expires = jiffies +
		usecs_to_jiffies(tunables->timer_rate);
	unsigned long flags;
	u64 now = ktime_to_us(ktime_get());

	pcpu->cpu_timer.expires = expires;
	add_timer_on(&pcpu->cpu_timer, cpu);
	if (tunables->timer_slack_val >= 0 &&
	    (pcpu->target_freq > pcpu->policy->min ||
		(pcpu->target_freq == pcpu->policy->min &&
		 now < tunables->boostpulse_endtime))) {
		expires += usecs_to_jiffies(tunables->timer_slack_val);
		pcpu->cpu_slack_timer.expires = expires;
		add_timer_on(&pcpu->cpu_slack_timer, cpu);
	}

	spin_lock_irqsave(&pcpu->load_lock, flags);
	pcpu->time_in_idle =
		get_cpu_idle_time(cpu, &pcpu->time_in_idle_timestamp,
				  tunables->io_is_busy);
	pcpu->cputime_speedadj = 0;
	pcpu->cputime_speedadj_timestamp = pcpu->time_in_idle_timestamp;
	spin_unlock_irqrestore(&pcpu->load_lock, flags);
}

static unsigned int freq_to_above_hispeed_delay(
	struct cpufreq_gabriel_plus_tunables *tunables,
	unsigned int freq)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->nabove_hispeed_delay - 1 &&
			freq >= tunables->above_hispeed_delay[i+1]; i += 2)
		;

	ret = tunables->above_hispeed_delay[i];
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);
	return ret;
}

static unsigned int freq_to_targetload(
	struct cpufreq_gabriel_plus_tunables *tunables, unsigned int freq)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads - 1 &&
		    freq >= tunables->target_loads[i+1]; i += 2)
		;

	ret = tunables->target_loads[i];
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

static unsigned int choose_target_freq(struct cpufreq_policy *policy,
					int index, unsigned int step, bool isup)
{
	struct cpufreq_frequency_table *table;
	unsigned int target_freq = 0;
	int i = 0;

	if (!policy || !step)
		return 0;

	table = policy->freq_table;
	if (isup) {
		for (i = (index + 1); (table[i].frequency != CPUFREQ_TABLE_END); i++) {
			if (table[i].frequency != CPUFREQ_ENTRY_INVALID) {
				target_freq = table[i].frequency;
				step--;
				if (step == 0) {
					break;
				}
			}
		}
	} else {
		for (i = (index - 1); i >= 0; i--) {
			if (table[i].frequency != CPUFREQ_ENTRY_INVALID) {
				target_freq = table[i].frequency;
				step--;
				if (step == 0) {
					break;
				}
			}
		}
	}
	return target_freq;
}

static u64 update_load(int cpu)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	struct cpufreq_gabriel_plus_tunables *tunables =
		pcpu->policy->governor_data;
	u64 now;
	u64 now_idle;
	u64 delta_idle;
	u64 delta_time;
	u64 active_time;

	now_idle = get_cpu_idle_time(cpu, &now, tunables->io_is_busy);
	delta_idle = (now_idle - pcpu->time_in_idle);
	delta_time = (now - pcpu->time_in_idle_timestamp);

	if (delta_time <= delta_idle)
		active_time = 0;
	else
		active_time = delta_time - delta_idle;

#ifdef CONFIG_LOAD_BASED_CORE_CURRENT_CAL
	if(pcpu->pre_cpu_for_load == pcpu->policy->cur) {
		pcpu->curr_speed_total_time += delta_time;
		pcpu->curr_speed_idle_time += delta_idle;
	} else {
		pcpu->curr_speed_total_time = delta_time;
		pcpu->curr_speed_idle_time = delta_idle;
	}
	pcpu->pre_cpu_for_load = pcpu->policy->cur;
#endif
	pcpu->cputime_speedadj += active_time * pcpu->policy->cur;

#if defined(CONFIG_CPU_THERMAL_IPA) || defined(CONFIG_EXYNOS_HOTPLUG_GOVERNOR)
	update_cpu_metric(cpu, now, delta_idle, delta_time, pcpu->policy);
#endif

	pcpu->time_in_idle = now_idle;
	pcpu->time_in_idle_timestamp = now;
	return now;
}

#ifdef CONFIG_LOAD_BASED_CORE_CURRENT_CAL
 unsigned int get_cpu_load(int cpu)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	unsigned int active_time, total_time;

	if((pcpu == NULL) || (pcpu->curr_speed_total_time == 0) 
			||(pcpu->curr_speed_idle_time > pcpu->curr_speed_total_time)) {
		return 0;
	}

	active_time = (unsigned int)(pcpu->curr_speed_total_time - pcpu->curr_speed_idle_time);
	total_time = (unsigned int)(pcpu->curr_speed_total_time);
	return (active_time*100) / total_time; 
}
#endif

static void cpufreq_gabriel_plus_timer(unsigned long data)
{
	u64 now;
	unsigned int delta_time;
	u64 cputime_speedadj;
	int cpu_load;
	struct cpufreq_gabriel_plus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	struct cpufreq_gabriel_plus_tunables *tunables =
		pcpu->policy->governor_data;
	unsigned int freq_calc_thresh = tunables->freq_calc_thresh;
	unsigned int timer_rate_idle = tunables->timer_rate_idle;
	unsigned int timer_rate_idle_freq = tunables->timer_rate_idle_freq;
	unsigned int idle_threshold = tunables->idle_threshold;
	unsigned int max_local_load = tunables->max_local_load;
	unsigned int bump_freq_weight;
	unsigned int avg_near_prev_load, avg_long_prev_load;
	unsigned int freq_responsiveness = tunables->freq_responsiveness;
	int pump_inc_step = tunables->pump_inc_step;
	int pump_dec_step = tunables->pump_dec_step;
	unsigned int load_idx;
	unsigned int new_freq;
	unsigned int loadadjfreq;
	unsigned int index;
	unsigned long flags;
	unsigned int this_hispeed_freq;
	u64 max_fvtime;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled)
		goto exit;

	spin_lock_irqsave(&pcpu->load_lock, flags);
	now = update_load(data);
	delta_time = (unsigned int)(now - pcpu->cputime_speedadj_timestamp);
	cputime_speedadj = pcpu->cputime_speedadj;
	spin_unlock_irqrestore(&pcpu->load_lock, flags);

	if (WARN_ON_ONCE(!delta_time))
		goto rearm;

	/*
	 * Average load of past five sampling. If avg_long_pre_load is lower
	 * than 20, the system is idle and should not be interrupted by
	 * CPUFreq governor timer.
	 */
	avg_long_prev_load = (pcpu->prev_load[4] +
			      pcpu->prev_load[3] +
			      pcpu->prev_load[2] +
			      pcpu->prev_load[1] +
			      pcpu->prev_load[0] + 4)/5;
	/*update the history load*/
	load_idx = pcpu->prev_load_idx;
	pcpu->prev_load_idx = (load_idx + 1)%5;
	pcpu->prev_load[pcpu->prev_load_idx] = cpu_load;

	/*switch timer to timer_rate_idle when system is idle to save power*/
	if (pcpu->policy->cur == pcpu->policy->min
		&& avg_long_prev_load <= idle_threshold
		&& cpu_load <= idle_threshold
		&& pcpu->policy->cur <= timer_rate_idle_freq)
		tunables->timer_rate = timer_rate_idle;
	else
		tunables->timer_rate = tunables->prev_timer_rate;

	/* CPUs Online Scale Frequency*/
	if (pcpu->policy->cur < freq_responsiveness) {
		pump_inc_step = tunables->pump_inc_step_at_min_freq;
		pump_dec_step = tunables->pump_dec_step_at_min_freq;
	}

	index = cpufreq_frequency_table_get_index(pcpu->policy, pcpu->policy->cur);
	if (index < 0) {
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	spin_lock_irqsave(&pcpu->target_freq_lock, flags);
	do_div(cputime_speedadj, delta_time);
	loadadjfreq = (unsigned int)cputime_speedadj * 100;
	cpu_load = loadadjfreq / pcpu->policy->cur;
	tunables->boosted = tunables->boost_val || now < tunables->boostpulse_endtime;
	this_hispeed_freq = max(tunables->hispeed_freq, pcpu->policy->min);

	if (cpu_load >= tunables->go_hispeed_load || tunables->boosted) {
		if (pcpu->policy->cur < this_hispeed_freq &&
		    cpu_load <= tunables->max_local_load) {
//			new_freq = pcpu->policy->cur * bump_freq_weight / 100;
			new_freq = this_hispeed_freq * bump_freq_weight / 100;
		} else {
			new_freq = choose_target_freq(pcpu->policy,
				index, pump_inc_step, true);

			if (new_freq > tunables->freq_calc_thresh)
				new_freq = pcpu->policy->max * cpu_load / 100;

			if (new_freq < this_hispeed_freq)
				new_freq = this_hispeed_freq;
		}
	} else {
		new_freq = choose_target_freq(pcpu->policy,
				index, pump_dec_step, false);
		if (new_freq > tunables->hispeed_freq &&
				pcpu->policy->cur < tunables->hispeed_freq)
			new_freq = tunables->hispeed_freq;

		if (new_freq > tunables->freq_calc_thresh)
			new_freq = pcpu->policy->max * cpu_load / 100;
	}

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_L,
					   &index)) {
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	if (cpu_load <= tunables->max_local_load &&
	    pcpu->policy->cur >= this_hispeed_freq &&
	    new_freq > pcpu->policy->cur &&
	    now - pcpu->pol_hispeed_val_time <
	    freq_to_above_hispeed_delay(tunables, pcpu->policy->cur)) {
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto target_update;
	}

	if (pcpu->target_freq >= pcpu->policy->max
	    && new_freq < pcpu->target_freq
	    && now - pcpu->max_freq_hyst_start_time <
	    tunables->max_freq_hysteresis) {
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	pcpu->loc_hispeed_val_time = now;

	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */
	max_fvtime = max(pcpu->pol_floor_val_time, pcpu->loc_floor_val_time);
	if (new_freq < pcpu->floor_freq &&
	    pcpu->target_freq >= pcpu->policy->cur) {
		if (now - max_fvtime < tunables->min_sample_time) {
			spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
			goto rearm;
		}
	}

	/*
	 * Update the timestamp for checking whether speed has been held at
	 * or above the selected frequency for a minimum of min_sample_time,
	 * if not boosted to this_hispeed_freq. If boosted to this_hispeed_freq
	 * then we allow the speed to drop as soon as the boostpulse duration
	 * expires (or the indefinite boost is turned off).
	 */

	if (!tunables->boosted || new_freq > this_hispeed_freq) {
		pcpu->floor_freq = new_freq;
		if (pcpu->target_freq >= pcpu->policy->cur ||
		    new_freq >= pcpu->policy->cur)
			pcpu->loc_floor_val_time = now;
	}

	if (pcpu->target_freq == new_freq &&
			pcpu->target_freq <= pcpu->policy->cur) {
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
		goto rearm;
	}

	pcpu->target_freq = new_freq;
	spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
	spin_lock_irqsave(&speedchange_cpumask_lock, flags);
	cpumask_set_cpu(data, &speedchange_cpumask);
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);
	wake_up_process(speedchp_task);

	goto rearm;

target_update:
	pcpu->target_freq = pcpu->policy->cur;

rearm:
	if (!timer_pending(&pcpu->cpu_timer))
		cpufreq_gabriel_plus_timer_resched(pcpu);

exit:
	up_read(&pcpu->enable_sem);
	return;
}

static void cpufreq_gabriel_plus_idle_end(void)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		return;
	}

	/* Arm the timer for 1-2 ticks later if not already. */
	if (!timer_pending(&pcpu->cpu_timer)) {
		cpufreq_gabriel_plus_timer_resched(pcpu);
	} else if (time_after_eq(jiffies, pcpu->cpu_timer.expires)) {
		del_timer(&pcpu->cpu_timer);
		del_timer(&pcpu->cpu_slack_timer);
		cpufreq_gabriel_plus_timer(smp_processor_id());
	}

	up_read(&pcpu->enable_sem);
}

static int cpufreq_gabriel_plus_speedchp_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&speedchange_cpumask_lock, flags);

		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
					       flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);

		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq = 0;
			struct cpufreq_gabriel_plus_cpuinfo *pjcpu;
			u64 hvt = ~0ULL, fvt = 0;

			pcpu = &per_cpu(cpuinfo, cpu);
			if (!down_read_trylock(&pcpu->enable_sem))
				continue;
			if (!pcpu->governor_enabled) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			for_each_cpu(j, pcpu->policy->cpus) {
				pjcpu = &per_cpu(cpuinfo, j);

				fvt = max(fvt, pjcpu->loc_floor_val_time);
				if (pjcpu->target_freq > max_freq) {
					max_freq = pjcpu->target_freq;
					hvt = pjcpu->loc_hispeed_val_time;
				} else if (pjcpu->target_freq == max_freq) {
					hvt = min(hvt, pjcpu->loc_hispeed_val_time);
				}
			}
			for_each_cpu(j, pcpu->policy->cpus) {
				pjcpu = &per_cpu(cpuinfo, j);
				pjcpu->pol_floor_val_time = fvt;
			}

			if (max_freq != pcpu->policy->cur) {
				__cpufreq_driver_target(pcpu->policy,
							max_freq,
							CPUFREQ_RELATION_H);
				for_each_cpu(j, pcpu->policy->cpus) {
					pjcpu = &per_cpu(cpuinfo, j);
					pjcpu->pol_hispeed_val_time = hvt;
				}
			}

#if defined(CONFIG_CPU_THERMAL_IPA)
			ipa_cpufreq_requested(pcpu->policy, max_freq);
#endif

			up_read(&pcpu->enable_sem);
		}
	}

	return 0;
}

static void cpufreq_gabriel_plus_boost(struct cpufreq_gabriel_plus_tunables *tunables)
{
	int i;
	int anyboost = 0;
	unsigned long flags[2];
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	struct cpumask boost_mask;
	struct cpufreq_policy *policy = container_of(tunables->policy,
						struct cpufreq_policy, policy);

	tunables->boosted = true;

	spin_lock_irqsave(&speedchange_cpumask_lock, flags[0]);

	if (have_governor_per_policy())
		cpumask_copy(&boost_mask, policy->cpus);
	else
		cpumask_copy(&boost_mask, cpu_online_mask);

	for_each_cpu(i, &boost_mask) {
		pcpu = &per_cpu(cpuinfo, i);
		if (tunables != pcpu->policy->governor_data)
			continue;

		spin_lock_irqsave(&pcpu->target_freq_lock, flags[1]);
		if (pcpu->target_freq < tunables->hispeed_freq) {
			pcpu->target_freq = tunables->hispeed_freq;
			cpumask_set_cpu(i, &speedchange_cpumask);
			pcpu->pol_hispeed_val_time =
				ktime_to_us(ktime_get());
			anyboost = 1;
		}
		spin_unlock_irqrestore(&pcpu->target_freq_lock, flags[1]);
	}

	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags[0]);

	if (anyboost && speedchp_task)
		wake_up_process(speedchp_task);
}

static int cpufreq_gabriel_plus_notifier(
	struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	int cpu;
	unsigned long flags;

	if (val == CPUFREQ_POSTCHANGE) {
		pcpu = &per_cpu(cpuinfo, freq->cpu);
		if (!down_read_trylock(&pcpu->enable_sem))
			return 0;
		if (!pcpu->governor_enabled) {
			up_read(&pcpu->enable_sem);
			return 0;
		}

		for_each_cpu(cpu, pcpu->policy->cpus) {
			struct cpufreq_gabriel_plus_cpuinfo *pjcpu =
				&per_cpu(cpuinfo, cpu);
			if (cpu != freq->cpu) {
				if (!down_read_trylock(&pjcpu->enable_sem))
					continue;
				if (!pjcpu->governor_enabled) {
					up_read(&pjcpu->enable_sem);
					continue;
				}
			}
			spin_lock_irqsave(&pjcpu->load_lock, flags);
			update_load(cpu);
			spin_unlock_irqrestore(&pjcpu->load_lock, flags);
			if (cpu != freq->cpu)
				up_read(&pjcpu->enable_sem);
		}

		up_read(&pcpu->enable_sem);
	}
	return 0;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_gabriel_plus_notifier,
};

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;
	return tokenized_data;

err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t show_target_loads(
	struct cpufreq_gabriel_plus_tunables *tunables,
	char *buf)
{
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&tunables->target_loads_lock, flags);

	for (i = 0; i < tunables->ntarget_loads; i++)
		ret += sprintf(buf + ret, "%u%s", tunables->target_loads[i],
			       i & 0x1 ? ":" : " ");

	sprintf(buf + ret - 1, "\n");
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return ret;
}

static ssize_t store_target_loads(
	struct cpufreq_gabriel_plus_tunables *tunables,
	const char *buf, size_t count)
{
	int ntokens;
	unsigned int *new_target_loads = NULL;
	unsigned long flags;

	new_target_loads = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_target_loads))
		return PTR_RET(new_target_loads);

	spin_lock_irqsave(&tunables->target_loads_lock, flags);
	if (tunables->target_loads != default_target_loads)
		kfree(tunables->target_loads);
	tunables->target_loads = new_target_loads;
	tunables->ntarget_loads = ntokens;
	spin_unlock_irqrestore(&tunables->target_loads_lock, flags);
	return count;
}

static ssize_t show_above_hispeed_delay(
	struct cpufreq_gabriel_plus_tunables *tunables, char *buf)
{
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->nabove_hispeed_delay; i++)
		ret += sprintf(buf + ret, "%u%s",
			       tunables->above_hispeed_delay[i],
			       i & 0x1 ? ":" : " ");

	sprintf(buf + ret - 1, "\n");
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);
	return ret;
}

static ssize_t store_above_hispeed_delay(
	struct cpufreq_gabriel_plus_tunables *tunables,
	const char *buf, size_t count)
{
	int ntokens;
	unsigned int *new_above_hispeed_delay = NULL;
	unsigned long flags;

	new_above_hispeed_delay = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_above_hispeed_delay))
		return PTR_RET(new_above_hispeed_delay);

	spin_lock_irqsave(&tunables->above_hispeed_delay_lock, flags);
	if (tunables->above_hispeed_delay != default_above_hispeed_delay)
		kfree(tunables->above_hispeed_delay);
	tunables->above_hispeed_delay = new_above_hispeed_delay;
	tunables->nabove_hispeed_delay = ntokens;
	spin_unlock_irqrestore(&tunables->above_hispeed_delay_lock, flags);
	return count;

}

static ssize_t show_hispeed_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%u\n", tunables->hispeed_freq);
}

static ssize_t store_hispeed_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	long unsigned int val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->hispeed_freq = val;
	return count;
}

static ssize_t show_freq_calc_thresh(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%u\n", tunables->freq_calc_thresh);
}

static ssize_t store_freq_calc_thresh(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	long unsigned int val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->freq_calc_thresh = val;
	return count;
}

#define show_store_one(file_name)					\
static ssize_t show_##file_name(					\
	struct cpufreq_gabriel_plus_tunables *tunables, char *buf)	\
{									\
	return snprintf(buf, PAGE_SIZE, "%u\n", tunables->file_name);	\
}									\
static ssize_t store_##file_name(					\
		struct cpufreq_gabriel_plus_tunables *tunables,		\
		const char *buf, size_t count)				\
{									\
	int ret;							\
	unsigned long int val;						\
									\
	ret = kstrtoul(buf, 0, &val);				\
	if (ret < 0)							\
		return ret;						\
	tunables->file_name = val;					\
	return count;							\
}
show_store_one(max_freq_hysteresis);

static ssize_t show_go_hispeed_load(struct cpufreq_gabriel_plus_tunables
		*tunables, char *buf)
{
	return sprintf(buf, "%lu\n", tunables->go_hispeed_load);
}

static ssize_t store_go_hispeed_load(struct cpufreq_gabriel_plus_tunables
		*tunables, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->go_hispeed_load = val;
	return count;
}

static ssize_t show_min_sample_time(struct cpufreq_gabriel_plus_tunables
		*tunables, char *buf)
{
	return sprintf(buf, "%lu\n", tunables->min_sample_time);
}

static ssize_t store_min_sample_time(struct cpufreq_gabriel_plus_tunables
		*tunables, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->min_sample_time = val;
	return count;
}

static ssize_t show_timer_rate(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%lu\n", tunables->timer_rate);
}

static ssize_t store_timer_rate(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	unsigned long val, val_round;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	val_round = jiffies_to_usecs(usecs_to_jiffies(val));
	if (val != val_round)
		pr_warn("timer_rate not aligned to jiffy. Rounded up to %lu\n",
			val_round);

	tunables->timer_rate = val_round;

	return count;
}

static ssize_t show_timer_slack(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->timer_slack_val);
}

static ssize_t store_timer_slack(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	tunables->timer_slack_val = val;
	return count;
}

static ssize_t show_boost(struct cpufreq_gabriel_plus_tunables *tunables,
			  char *buf)
{
	return sprintf(buf, "%d\n", tunables->boost_val);
}

static ssize_t store_boost(struct cpufreq_gabriel_plus_tunables *tunables,
			   const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->boost_val = val;

	if (tunables->boost_val) {
		if (!tunables->boosted)
			cpufreq_gabriel_plus_boost(tunables);
	} else {
		tunables->boostpulse_endtime = ktime_to_us(ktime_get());
	}

	return count;
}

static ssize_t store_boostpulse(struct cpufreq_gabriel_plus_tunables *tunables,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->boostpulse_endtime = ktime_to_us(ktime_get()) +
		tunables->boostpulse_duration_val;
	if (!tunables->boosted)
		cpufreq_gabriel_plus_boost(tunables);
	return count;
}

static ssize_t show_boostpulse_duration(struct cpufreq_gabriel_plus_tunables
		*tunables, char *buf)
{
	return sprintf(buf, "%d\n", tunables->boostpulse_duration_val);
}

static ssize_t store_boostpulse_duration(struct cpufreq_gabriel_plus_tunables
		*tunables, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	tunables->boostpulse_duration_val = val;
	return count;
}

static ssize_t show_io_is_busy(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%u\n", tunables->io_is_busy);
}

static ssize_t store_io_is_busy(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	/* Prevent ROM to set 1 here */
	if (val == 1)
		val = 0;
	tunables->io_is_busy = val;
	return count;
}

static ssize_t show_align_windows(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%u\n", tunables->align_windows);
}

static ssize_t store_align_windows(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->align_windows = val;
	return count;
}

static ssize_t show_timer_rate_idle(struct cpufreq_gabriel_plus_tunables
               *tunables, char *buf)
{
       return sprintf(buf, "%lu\n", tunables->timer_rate_idle);
}

static ssize_t store_timer_rate_idle(struct cpufreq_gabriel_plus_tunables
               *tunables, const char *buf, size_t count)
{
       int ret;
       unsigned long val, val_round;

       ret = kstrtoul(buf, 0, &val);
       if (ret < 0)
               return ret;

       val_round = jiffies_to_usecs(usecs_to_jiffies(val));
       if (val != val_round)
               pr_warn("timer_rate_idle not aligned to jiffy. Rounded up to %lu\n",
                       val_round);

       tunables->timer_rate_idle = val_round;

       return count;
}

static ssize_t show_timer_rate_idle_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%lu\n", tunables->timer_rate_idle_freq);
}

static ssize_t store_timer_rate_idle_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int ret;
	long unsigned int val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->timer_rate_idle_freq = val;
	return count;
}

static ssize_t show_idle_threshold(struct cpufreq_gabriel_plus_tunables
		*tunables, char *buf)
{
	return sprintf(buf, "%lu\n", tunables->idle_threshold);
}

static ssize_t store_idle_threshold(struct cpufreq_gabriel_plus_tunables
		*tunables, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->idle_threshold = val;
	return count;
}

static ssize_t show_max_local_load(struct cpufreq_gabriel_plus_tunables
		*tunables, char *buf)
{
	return sprintf(buf, "%lu\n", tunables->max_local_load);
}

static ssize_t store_max_local_load(struct cpufreq_gabriel_plus_tunables
		*tunables, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->max_local_load = val;
	return count;
}

/* freq_responsiveness */
static ssize_t show_freq_responsiveness(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->freq_responsiveness);
}

static ssize_t store_freq_responsiveness(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	if (input == tunables->freq_responsiveness)
		return count;

	tunables->freq_responsiveness = input;

	return count;
}

/* pump_inc_step_at_min_freq */
static ssize_t show_pump_inc_step_at_min_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->pump_inc_step_at_min_freq);
}

static ssize_t store_pump_inc_step_at_min_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = min(max(1, input), 6);

	if (input == tunables->pump_inc_step_at_min_freq)
		return count;

	tunables->pump_inc_step_at_min_freq = input;

	return count;
}

/* pump_inc_step */
static ssize_t show_pump_inc_step(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->pump_inc_step);
}

static ssize_t store_pump_inc_step(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = min(max(1, input), 6);

	if (input == tunables->pump_inc_step)
		return count;

	tunables->pump_inc_step = input;

	return count;
}

/* pump_dec_step_at_min_freq */
static ssize_t show_pump_dec_step_at_min_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->pump_dec_step_at_min_freq);
}

static ssize_t store_pump_dec_step_at_min_freq(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = min(max(1, input), 6);

	if (input == tunables->pump_dec_step_at_min_freq)
		return count;

	tunables->pump_dec_step_at_min_freq = input;

	return count;
}

/* pump_dec_step */
static ssize_t show_pump_dec_step(struct cpufreq_gabriel_plus_tunables *tunables,
		char *buf)
{
	return sprintf(buf, "%d\n", tunables->pump_dec_step);
}

static ssize_t store_pump_dec_step(struct cpufreq_gabriel_plus_tunables *tunables,
		const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = min(max(1, input), 6);

	if (input == tunables->pump_dec_step)
		return count;

	tunables->pump_dec_step = input;

	return count;
}

static ssize_t show_bump_freq_weight(struct cpufreq_gabriel_plus_tunables
		*tunables, char *buf)
{
	return sprintf(buf, "%lu\n", tunables->bump_freq_weight);
}

static ssize_t store_bump_freq_weight(struct cpufreq_gabriel_plus_tunables
		*tunables, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	tunables->bump_freq_weight = val;
	return count;
}

/*
 * Create show/store routines
 * - sys: One governor instance for complete SYSTEM
 * - pol: One governor instance per struct cpufreq_policy
 */
#define show_gov_pol_sys(file_name)					\
static ssize_t show_##file_name##_gov_sys				\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return show_##file_name(common_tunables, buf);			\
}									\
									\
static ssize_t show_##file_name##_gov_pol				\
(struct cpufreq_policy *policy, char *buf)				\
{									\
	return show_##file_name(policy->governor_data, buf);		\
}

#define store_gov_pol_sys(file_name)					\
static ssize_t store_##file_name##_gov_sys				\
(struct kobject *kobj, struct attribute *attr, const char *buf,		\
	size_t count)							\
{									\
	return store_##file_name(common_tunables, buf, count);		\
}									\
									\
static ssize_t store_##file_name##_gov_pol				\
(struct cpufreq_policy *policy, const char *buf, size_t count)		\
{									\
	return store_##file_name(policy->governor_data, buf, count);	\
}

#define show_store_gov_pol_sys(file_name)				\
show_gov_pol_sys(file_name);						\
store_gov_pol_sys(file_name)

show_store_gov_pol_sys(target_loads);
show_store_gov_pol_sys(above_hispeed_delay);
show_store_gov_pol_sys(hispeed_freq);
show_store_gov_pol_sys(freq_calc_thresh);
show_store_gov_pol_sys(go_hispeed_load);
show_store_gov_pol_sys(min_sample_time);
show_store_gov_pol_sys(timer_rate);
show_store_gov_pol_sys(timer_rate_idle);
show_store_gov_pol_sys(timer_rate_idle_freq);
show_store_gov_pol_sys(idle_threshold);
show_store_gov_pol_sys(max_local_load);
show_store_gov_pol_sys(bump_freq_weight);
show_store_gov_pol_sys(timer_slack);
show_store_gov_pol_sys(boost);
store_gov_pol_sys(boostpulse);
show_store_gov_pol_sys(boostpulse_duration);
show_store_gov_pol_sys(io_is_busy);
show_store_gov_pol_sys(align_windows);
show_store_gov_pol_sys(max_freq_hysteresis);
show_store_gov_pol_sys(freq_responsiveness);
show_store_gov_pol_sys(pump_inc_step_at_min_freq);
show_store_gov_pol_sys(pump_inc_step);
show_store_gov_pol_sys(pump_dec_step_at_min_freq);
show_store_gov_pol_sys(pump_dec_step);

#define gov_sys_attr_rw(_name)						\
static struct global_attr _name##_gov_sys =				\
__ATTR(_name, 0664, show_##_name##_gov_sys, store_##_name##_gov_sys)

#define gov_pol_attr_rw(_name)						\
static struct freq_attr _name##_gov_pol =				\
__ATTR(_name, 0664, show_##_name##_gov_pol, store_##_name##_gov_pol)

#define gov_sys_pol_attr_rw(_name)					\
	gov_sys_attr_rw(_name);						\
	gov_pol_attr_rw(_name)

gov_sys_pol_attr_rw(target_loads);
gov_sys_pol_attr_rw(above_hispeed_delay);
gov_sys_pol_attr_rw(hispeed_freq);
gov_sys_pol_attr_rw(freq_calc_thresh);
gov_sys_pol_attr_rw(go_hispeed_load);
gov_sys_pol_attr_rw(min_sample_time);
gov_sys_pol_attr_rw(timer_rate);
gov_sys_pol_attr_rw(timer_rate_idle);
gov_sys_pol_attr_rw(timer_rate_idle_freq);
gov_sys_pol_attr_rw(idle_threshold);
gov_sys_pol_attr_rw(max_local_load);
gov_sys_pol_attr_rw(bump_freq_weight);
gov_sys_pol_attr_rw(timer_slack);
gov_sys_pol_attr_rw(boost);
gov_sys_pol_attr_rw(boostpulse_duration);
gov_sys_pol_attr_rw(io_is_busy);
gov_sys_pol_attr_rw(align_windows);
gov_sys_pol_attr_rw(max_freq_hysteresis);
gov_sys_pol_attr_rw(freq_responsiveness);
gov_sys_pol_attr_rw(pump_inc_step_at_min_freq);
gov_sys_pol_attr_rw(pump_inc_step);
gov_sys_pol_attr_rw(pump_dec_step_at_min_freq);
gov_sys_pol_attr_rw(pump_dec_step);

static struct global_attr boostpulse_gov_sys =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse_gov_sys);

static struct freq_attr boostpulse_gov_pol =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse_gov_pol);

/* One Governor instance for entire system */
static struct attribute *gabriel_plus_attributes_gov_sys[] = {
	&target_loads_gov_sys.attr,
	&above_hispeed_delay_gov_sys.attr,
	&hispeed_freq_gov_sys.attr,
	&freq_calc_thresh_gov_sys.attr,
	&go_hispeed_load_gov_sys.attr,
	&min_sample_time_gov_sys.attr,
	&timer_rate_gov_sys.attr,
	&timer_rate_idle_gov_sys.attr,
	&timer_rate_idle_freq_gov_sys.attr,
	&idle_threshold_gov_sys.attr,
	&max_local_load_gov_sys.attr,
	&bump_freq_weight_gov_sys.attr,
	&timer_slack_gov_sys.attr,
	&boost_gov_sys.attr,
	&boostpulse_gov_sys.attr,
	&boostpulse_duration_gov_sys.attr,
	&io_is_busy_gov_sys.attr,
	&align_windows_gov_sys.attr,
	&max_freq_hysteresis_gov_sys.attr,
	&freq_responsiveness_gov_sys.attr,
	&pump_inc_step_at_min_freq_gov_sys.attr,
	&pump_inc_step_gov_sys.attr,
	&pump_dec_step_at_min_freq_gov_sys.attr,
	&pump_dec_step_gov_sys.attr,
	NULL,
};

static struct attribute_group gabriel_plus_attr_group_gov_sys = {
	.attrs = gabriel_plus_attributes_gov_sys,
	.name = "gabriel_plus",
};

/* Per policy governor instance */
static struct attribute *gabriel_plus_attributes_gov_pol[] = {
	&target_loads_gov_pol.attr,
	&above_hispeed_delay_gov_pol.attr,
	&hispeed_freq_gov_pol.attr,
	&freq_calc_thresh_gov_pol.attr,
	&go_hispeed_load_gov_pol.attr,
	&min_sample_time_gov_pol.attr,
	&timer_rate_gov_pol.attr,
	&timer_rate_idle_gov_pol.attr,
	&timer_rate_idle_freq_gov_pol.attr,
	&idle_threshold_gov_pol.attr,
	&max_local_load_gov_pol.attr,
	&bump_freq_weight_gov_pol.attr,
	&timer_slack_gov_pol.attr,
	&boost_gov_pol.attr,
	&boostpulse_gov_pol.attr,
	&boostpulse_duration_gov_pol.attr,
	&io_is_busy_gov_pol.attr,
	&align_windows_gov_pol.attr,
	&max_freq_hysteresis_gov_pol.attr,
	&freq_responsiveness_gov_pol.attr,
	&pump_inc_step_at_min_freq_gov_pol.attr,
	&pump_inc_step_gov_pol.attr,
	&pump_dec_step_at_min_freq_gov_pol.attr,
	&pump_dec_step_gov_pol.attr,
	NULL,
};

static struct attribute_group gabriel_plus_attr_group_gov_pol = {
	.attrs = gabriel_plus_attributes_gov_pol,
	.name = "gabriel_plus",
};

static struct attribute_group *get_sysfs_attr(void)
{
	if (have_governor_per_policy())
		return &gabriel_plus_attr_group_gov_pol;
	else
		return &gabriel_plus_attr_group_gov_sys;
}

static int cpufreq_gabriel_plus_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	if (val == IDLE_END)
		cpufreq_gabriel_plus_idle_end();

	return 0;
}

static struct notifier_block cpufreq_gabriel_plus_idle_nb = {
	.notifier_call = cpufreq_gabriel_plus_idle_notifier,
};

static int cpufreq_governor_gabriel_plus(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j;
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;
	struct cpufreq_gabriel_plus_tunables *tunables;
	unsigned long flags;

	if (have_governor_per_policy())
		tunables = policy->governor_data;
	else
		tunables = common_tunables;

	WARN_ON(!tunables && (event != CPUFREQ_GOV_POLICY_INIT));

	switch (event) {
	case CPUFREQ_GOV_POLICY_INIT:
		if (have_governor_per_policy()) {
			WARN_ON(tunables);
		} else if (tunables) {
			tunables->usage_count++;
			policy->governor_data = tunables;
			return 0;
		}

		tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
		if (!tunables) {
			pr_err("%s: POLICY_INIT: kzalloc failed\n", __func__);
			return -ENOMEM;
		}

		if (!tuned_parameters[policy->cpu]) {
			tunables->above_hispeed_delay = default_above_hispeed_delay;
			tunables->nabove_hispeed_delay =
				ARRAY_SIZE(default_above_hispeed_delay);
			tunables->go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
			tunables->hispeed_freq = DEFAULT_HISPEED_FREQ;
			tunables->target_loads = default_target_loads;
			tunables->ntarget_loads = ARRAY_SIZE(default_target_loads);
			tunables->min_sample_time = DEFAULT_MIN_SAMPLE_TIME;
			tunables->timer_rate = DEFAULT_TIMER_RATE;
			tunables->prev_timer_rate = DEFAULT_TIMER_RATE;
			tunables->timer_rate_idle = DEFAULT_TIMER_RATE_IDLE;
			tunables->timer_rate_idle_freq = DEFAULT_TIMER_RATE_IDLE_FREQ;
			tunables->bump_freq_weight = DEFAULT_BUMP_FREQ_WEIGHT;
			tunables->idle_threshold= DEFAULT_IDLE_THRESHOLD;
			tunables->max_local_load= DEFAULT_MAX_LOCAL_LOAD;
			tunables->boostpulse_duration_val = DEFAULT_MIN_SAMPLE_TIME;
			tunables->timer_slack_val = DEFAULT_TIMER_SLACK;
			tunables->freq_responsiveness = FREQ_RESPONSIVENESS;
			tunables->pump_inc_step_at_min_freq = PUMP_INC_STEP_AT_MIN_FREQ;
			tunables->pump_inc_step = PUMP_INC_STEP;
			tunables->pump_dec_step = PUMP_DEC_STEP;
			tunables->pump_dec_step_at_min_freq = PUMP_DEC_STEP_AT_MIN_FREQ;
		} else {
			memcpy(tunables, tuned_parameters[policy->cpu], sizeof(*tunables));
			kfree(tuned_parameters[policy->cpu]);
		}
		tunables->usage_count = 1;

		/* update handle for get cpufreq_policy */
		tunables->policy = &policy->policy;

		spin_lock_init(&tunables->target_loads_lock);
		spin_lock_init(&tunables->above_hispeed_delay_lock);

		policy->governor_data = tunables;
		if (!have_governor_per_policy()) {
			common_tunables = tunables;
			WARN_ON(cpufreq_get_global_kobject());
		}

		rc = sysfs_create_group(get_governor_parent_kobj(policy),
				get_sysfs_attr());
		if (rc) {
			kfree(tunables);
			policy->governor_data = NULL;
			if (!have_governor_per_policy()) {
				common_tunables = NULL;
				cpufreq_put_global_kobject();
			}
			return rc;
		}

		if (!policy->governor->initialized) {
			idle_notifier_register(&cpufreq_gabriel_plus_idle_nb);
			cpufreq_register_notifier(&cpufreq_notifier_block,
					CPUFREQ_TRANSITION_NOTIFIER);
		}

		break;

	case CPUFREQ_GOV_POLICY_EXIT:
		if (!--tunables->usage_count) {
			if (policy->governor->initialized == 1) {
				cpufreq_unregister_notifier(&cpufreq_notifier_block,
						CPUFREQ_TRANSITION_NOTIFIER);
				idle_notifier_unregister(&cpufreq_gabriel_plus_idle_nb);
			}

			sysfs_remove_group(get_governor_parent_kobj(policy),
					get_sysfs_attr());

			if (!have_governor_per_policy())
				cpufreq_put_global_kobject();

			tuned_parameters[policy->cpu] = kzalloc(sizeof(*tunables), GFP_KERNEL);
			if (!tuned_parameters[policy->cpu]) {
				pr_err("%s: POLICY_EXIT: kzalloc failed\n", __func__);
				return -ENOMEM;
			}
			memcpy(tuned_parameters[policy->cpu], tunables, sizeof(*tunables));
			kfree(tunables);
			common_tunables = NULL;
		}

		policy->governor_data = NULL;
		break;

	case CPUFREQ_GOV_START:
		mutex_lock(&gov_lock);

		freq_table = cpufreq_frequency_get_table(policy->cpu);
		if (!tunables->hispeed_freq)
			tunables->hispeed_freq = policy->max;
		tunables->freq_calc_thresh = policy->cpuinfo.min_freq;

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->floor_freq = pcpu->target_freq;
			pcpu->pol_floor_val_time =
				ktime_to_us(ktime_get());
			pcpu->loc_floor_val_time = pcpu->pol_floor_val_time;
			pcpu->pol_hispeed_val_time = pcpu->pol_floor_val_time;
			pcpu->loc_hispeed_val_time = pcpu->pol_floor_val_time;
			down_write(&pcpu->enable_sem);
			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);
			cpufreq_gabriel_plus_timer_start(tunables, j);
			pcpu->governor_enabled = 1;
			up_write(&pcpu->enable_sem);
		}

		mutex_unlock(&gov_lock);
		break;

	case CPUFREQ_GOV_STOP:
		mutex_lock(&gov_lock);
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			down_write(&pcpu->enable_sem);
			pcpu->governor_enabled = 0;
			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);
			up_write(&pcpu->enable_sem);
		}

		mutex_unlock(&gov_lock);
		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);

			down_read(&pcpu->enable_sem);
			if (pcpu->governor_enabled == 0) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			spin_lock_irqsave(&pcpu->target_freq_lock, flags);
			if (policy->max < pcpu->target_freq)
				pcpu->target_freq = policy->max;
			else if (policy->min > pcpu->target_freq)
				pcpu->target_freq = policy->min;

			spin_unlock_irqrestore(&pcpu->target_freq_lock, flags);
			up_read(&pcpu->enable_sem);
		}
		break;
	}
	return 0;
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_GABRIEL_PLUS
static
#endif
struct cpufreq_governor cpufreq_gov_gabriel_plus = {
	.name = "gabriel_plus",
	.governor = cpufreq_governor_gabriel_plus,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void cpufreq_gabriel_plus_nop_timer(unsigned long data)
{
}

unsigned int cpufreq_gabriel_plus_get_hispeed_freq(int cpu)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu =
			&per_cpu(cpuinfo, cpu);
	struct cpufreq_gabriel_plus_tunables *tunables;

	if (pcpu && pcpu->policy)
		tunables = pcpu->policy->governor_data;
	else
		return 0;

	if (!tunables)
		return 0;

	return tunables->hispeed_freq;
}

#ifdef CONFIG_ARCH_EXYNOS
static int cpufreq_gabriel_plus_cluster1_min_qos_handler(struct notifier_block *b,
						unsigned long val, void *v)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	struct cpufreq_gabriel_plus_tunables *tunables;
	unsigned long flags;
	int ret = NOTIFY_OK;
#if defined(CONFIG_ARM_EXYNOS_MP_CPUFREQ)
	int cpu = NR_CLUST0_CPUS;
#else
	int cpu = 0;
#endif

	pcpu = &per_cpu(cpuinfo, cpu);

	mutex_lock(&gov_lock);
	down_read(&pcpu->enable_sem);
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		ret = NOTIFY_BAD;
		goto exit;
	}
	up_read(&pcpu->enable_sem);

	if (!pcpu->policy || !pcpu->policy->governor_data ||
		!pcpu->policy->user_policy.governor) {
		ret = NOTIFY_BAD;
		goto exit;
	}

	if (val < pcpu->policy->cur) {
		tunables = pcpu->policy->governor_data;

		spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		cpumask_set_cpu(cpu, &speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		if (speedchp_task)
			wake_up_process(speedchp_task);
	}
exit:
	mutex_unlock(&gov_lock);
	return ret;
}

static struct notifier_block cpufreq_gabriel_plus_cluster1_min_qos_notifier = {
	.notifier_call = cpufreq_gabriel_plus_cluster1_min_qos_handler,
};

static int cpufreq_gabriel_plus_cluster1_max_qos_handler(struct notifier_block *b,
						unsigned long val, void *v)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	struct cpufreq_gabriel_plus_tunables *tunables;
	unsigned long flags;
	int ret = NOTIFY_OK;
#if defined(CONFIG_ARM_EXYNOS_MP_CPUFREQ)
	int cpu = NR_CLUST0_CPUS;
#else
	int cpu = 0;
#endif

	pcpu = &per_cpu(cpuinfo, cpu);

	mutex_lock(&gov_lock);
	down_read(&pcpu->enable_sem);
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		ret =  NOTIFY_BAD;
		goto exit;
	}
	up_read(&pcpu->enable_sem);

	if (!pcpu->policy || !pcpu->policy->governor_data ||
		!pcpu->policy->user_policy.governor) {
		ret = NOTIFY_BAD;
		goto exit;
	}

	if (val > pcpu->policy->cur) {
		tunables = pcpu->policy->governor_data;

		spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		cpumask_set_cpu(cpu, &speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		if (speedchp_task)
			wake_up_process(speedchp_task);
	}
exit:
	mutex_unlock(&gov_lock);
	return ret;
}

static struct notifier_block cpufreq_gabriel_plus_cluster1_max_qos_notifier = {
	.notifier_call = cpufreq_gabriel_plus_cluster1_max_qos_handler,
};

#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
static int cpufreq_gabriel_plus_cluster0_min_qos_handler(struct notifier_block *b,
						unsigned long val, void *v)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	struct cpufreq_gabriel_plus_tunables *tunables;
	unsigned long flags;
	int ret = NOTIFY_OK;

	pcpu = &per_cpu(cpuinfo, 0);

	mutex_lock(&gov_lock);
	down_read(&pcpu->enable_sem);
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		ret = NOTIFY_BAD;
		goto exit;
	}
	up_read(&pcpu->enable_sem);

	if (!pcpu->policy || !pcpu->policy->governor_data ||
		!pcpu->policy->user_policy.governor) {
		ret = NOTIFY_BAD;
		goto exit;
	}

	if (val < pcpu->policy->cur) {
		tunables = pcpu->policy->governor_data;

		spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		cpumask_set_cpu(0, &speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		if (speedchp_task)
			wake_up_process(speedchp_task);
	}
exit:
	mutex_unlock(&gov_lock);
	return ret;
}

static struct notifier_block cpufreq_gabriel_plus_cluster0_min_qos_notifier = {
	.notifier_call = cpufreq_gabriel_plus_cluster0_min_qos_handler,
};

static int cpufreq_gabriel_plus_cluster0_max_qos_handler(struct notifier_block *b,
						unsigned long val, void *v)
{
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;
	struct cpufreq_gabriel_plus_tunables *tunables;
	unsigned long flags;
	int ret = NOTIFY_OK;

	pcpu = &per_cpu(cpuinfo, 0);

	mutex_lock(&gov_lock);
	down_read(&pcpu->enable_sem);
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		ret = NOTIFY_BAD;
		goto exit;
	}
	up_read(&pcpu->enable_sem);

	if (!pcpu->policy ||!pcpu->policy->governor_data ||
		!pcpu->policy->user_policy.governor) {
		ret = NOTIFY_BAD;
		goto exit;
	}

	if (val > pcpu->policy->cur) {
		tunables = pcpu->policy->governor_data;

		spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		cpumask_set_cpu(0, &speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		if (speedchp_task)
			wake_up_process(speedchp_task);
	}
exit:
	mutex_unlock(&gov_lock);
	return ret;
}

static struct notifier_block cpufreq_gabriel_plus_cluster0_max_qos_notifier = {
	.notifier_call = cpufreq_gabriel_plus_cluster0_max_qos_handler,
};
#endif
#endif

static int __init cpufreq_gabriel_plus_init(void)
{
	unsigned int i;
	struct cpufreq_gabriel_plus_cpuinfo *pcpu;

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer_deferrable(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_gabriel_plus_timer;
		pcpu->cpu_timer.data = i;
		init_timer(&pcpu->cpu_slack_timer);
		pcpu->cpu_slack_timer.function = cpufreq_gabriel_plus_nop_timer;
		spin_lock_init(&pcpu->load_lock);
		spin_lock_init(&pcpu->target_freq_lock);
		init_rwsem(&pcpu->enable_sem);
#ifdef CONFIG_LOAD_BASED_CORE_CURRENT_CAL
		pcpu->pre_cpu_for_load = 0;
		pcpu->curr_speed_total_time = 0;
		pcpu->curr_speed_idle_time = 0;
#endif
	}

	spin_lock_init(&speedchange_cpumask_lock);
	mutex_init(&gov_lock);

	speedchp_task =
		kthread_create(cpufreq_gabriel_plus_speedchp_task, NULL,
				"cfgabriel_plus");
	if (IS_ERR(speedchp_task))
		return PTR_ERR(speedchp_task);

	kthread_bind(speedchp_task, 0);

#ifdef CONFIG_ARCH_EXYNOS
	pm_qos_add_notifier(PM_QOS_CLUSTER1_FREQ_MIN, &cpufreq_gabriel_plus_cluster1_min_qos_notifier);
	pm_qos_add_notifier(PM_QOS_CLUSTER1_FREQ_MAX, &cpufreq_gabriel_plus_cluster1_max_qos_notifier);
#ifdef CONFIG_ARM_EXYNOS_MP_CPUFREQ
	pm_qos_add_notifier(PM_QOS_CLUSTER0_FREQ_MIN, &cpufreq_gabriel_plus_cluster0_min_qos_notifier);
	pm_qos_add_notifier(PM_QOS_CLUSTER0_FREQ_MAX, &cpufreq_gabriel_plus_cluster0_max_qos_notifier);
#endif
#endif

	return cpufreq_register_governor(&cpufreq_gov_gabriel_plus);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_GABRIEL_PLUS
fs_initcall(cpufreq_gabriel_plus_init);
#else
module_init(cpufreq_gabriel_plus_init);
#endif

static void __exit cpufreq_gabriel_plus_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_gabriel_plus);
}

module_exit(cpufreq_gabriel_plus_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_AUTHOR("Mostafa Zarghami <mostafazarghami.com>");
MODULE_DESCRIPTION("'cpufreq_gabriel_plus' - A cpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
