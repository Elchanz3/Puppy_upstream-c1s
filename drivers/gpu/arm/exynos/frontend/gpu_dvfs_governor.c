/* SPDX-License-Identifier: GPL-2.0 */
/*
 * (C) COPYRIGHT 2021 Samsung Electronics Inc. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 */

#include <gpex_dvfs.h>
#include <gpex_clock.h>
#include <gpex_pm.h>
#include <gpex_utils.h>
#include <gpex_clboost.h>
#include <gpex_tsg.h>

#include "gpex_dvfs_internal.h"
#include "gpu_dvfs_governor.h"

#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/thermal.h>
#include <linux/kernel.h>
#include <linux/ktime.h>

/* ------------------------------------------------------------------------- */
/*                          Profiles & Common Tunables                       */
/* ------------------------------------------------------------------------- */

enum governor_profile {
	GOV_PROFILE_SOFT = 0,
	GOV_PROFILE_MEDIUM,
	GOV_PROFILE_EXTREME,
	GOV_PROFILE_OVERCLOCK,
	GOV_PROFILE_MAX
};

static const char * const profile_names[] = {
	"soft", "medium", "extreme", "overclock"
};

static enum governor_profile current_profile = GOV_PROFILE_MEDIUM;

/* NOTE: Keep in ascending order (Hz). */
static const int freq_steps[] = {
	156000,  // 0
	260000,  // 1
	325000,  // 2
	377000,  // 3
	455000,  // 4
	481000,  // 5
	507000,  // 6
	572000,  // 7
	598000,  // 8
	702000,  // 9
	767000,  // 10
	799500,  // 11
	800000,  // 12
	832000,  // 13
	897000,  // 14
};
static const int num_steps = ARRAY_SIZE(freq_steps);

static int thermal_limit_temp = 82;     /* °C */
static int thermal_safe_step = 12;      /* ~800 MHz (index 12) */

static struct dvfs_info *dvfs;

/* ------------------------------------------------------------------------- */
/*                           Housekeeping Callbacks                           */
/* ------------------------------------------------------------------------- */

/* NOTE: Kept as-is, only formatting/clarity tweaks applied. */
int gpex_dvfs_set_clock_callback(void)
{
	unsigned long flags;
	int level = 0;
	int cur_clock = 0;

	cur_clock = gpex_clock_get_cur_clock();
	level = gpex_clock_get_table_idx(cur_clock);
	if (level >= 0) {
		spin_lock_irqsave(&dvfs->spinlock, flags);

		if (dvfs->step != level)
			dvfs->down_requirement = dvfs->table[level].down_staycount;

		if (dvfs->step < level)
			dvfs->interactive.delay_count = 0;

		dvfs->step = level;

		spin_unlock_irqrestore(&dvfs->spinlock, flags);
	} else {
		GPU_LOG(MALI_EXYNOS_ERROR, "%s: invalid dvfs level returned %d gpu power %d\n",
			__func__, cur_clock, gpex_pm_get_status(false));
		return -1;
	}
	return 0;
}

/* --------------------------- sysfs: profile ------------------------------ */

static ssize_t profile_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", profile_names[current_profile]);
}

static ssize_t profile_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	int i;
	for (i = 0; i < GOV_PROFILE_MAX; i++) {
		if (sysfs_streq(buf, profile_names[i])) {
			current_profile = i;
			pr_info("GPU governor: profile set to %s\n", profile_names[i]);
			return count;
		}
	}
	pr_warn("GPU governor: invalid profile '%s'\n", buf);
	return -EINVAL;
}
static DEVICE_ATTR_RW(profile);

/* ------------------------ sysfs: thermal limit --------------------------- */

static ssize_t thermal_limit_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", thermal_limit_temp);
}

static ssize_t thermal_limit_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int val;
	if (kstrtoint(buf, 10, &val) == 0) {
		thermal_limit_temp = val;
		pr_info("GPU governor: thermal limit set to %d°C\n", val);
		return count;
	}
	return -EINVAL;
}
static DEVICE_ATTR_RW(thermal_limit);

/* ------------------------------------------------------------------------- */
/*                               Governors                                   */
/* ------------------------------------------------------------------------- */

typedef int (*GET_NEXT_LEVEL)(int utilization);
static GET_NEXT_LEVEL gpu_dvfs_get_next_level;

static int gpu_dvfs_governor_energy_ondemand(int utilization);
static int gpu_dvfs_governor_interactive(int utilization);
static int gpu_dvfs_governor_joint(int utilization);
static int gpu_dvfs_governor_static(int utilization);
static int gpu_dvfs_governor_booster(int utilization);
static int gpu_dvfs_governor_dynamic(int utilization);
static int gpu_dvfs_governor_performance(int utilization);

static gpu_dvfs_governor_info governor_info[G3D_MAX_GOVERNOR_NUM] = {
	{
		G3D_DVFS_GOVERNOR_ENERGY_ONDEMAND,
		"Energy_ondemand",
		gpu_dvfs_governor_energy_ondemand,
	},
	{
		G3D_DVFS_GOVERNOR_INTERACTIVE,
		"Interactive",
		gpu_dvfs_governor_interactive,
	},
	{
		G3D_DVFS_GOVERNOR_JOINT,
		"Joint",
		gpu_dvfs_governor_joint,
	},
	{
		G3D_DVFS_GOVERNOR_STATIC,
		"Static_ondemand",
		gpu_dvfs_governor_static,
	},
	{
		G3D_DVFS_GOVERNOR_BOOSTER,
		"Booster",
		gpu_dvfs_governor_booster,
	},
	{
		G3D_DVFS_GOVERNOR_DYNAMIC,
		"Dynamic",
		gpu_dvfs_governor_dynamic,
	},
	{
		G3D_DVFS_GOVERNOR_PERFORMANCE,
		"Performance",
		gpu_dvfs_governor_performance,
	},
};

void gpu_dvfs_update_start_clk(int governor_type, int clk)
{
	governor_info[governor_type].start_clk = clk;
}

void *gpu_dvfs_get_governor_info(void)
{
	return &governor_info;
}

/* ------------------------------ Interactive ------------------------------ */

static int gpu_dvfs_governor_interactive(int utilization)
{
	if ((dvfs->step > gpex_clock_get_table_idx(gpex_clock_get_max_clock())) &&
	    (utilization > dvfs->table[dvfs->step].max_threshold)) {
		int highspeed_level = gpex_clock_get_table_idx(dvfs->interactive.highspeed_clock);
		if ((highspeed_level > 0) && (dvfs->step > highspeed_level) &&
		    (utilization > dvfs->interactive.highspeed_load)) {
			if (dvfs->interactive.delay_count == dvfs->interactive.highspeed_delay) {
				dvfs->step = highspeed_level;
				dvfs->interactive.delay_count = 0;
			} else {
				dvfs->interactive.delay_count++;
			}
		} else {
			dvfs->step--;
			dvfs->interactive.delay_count = 0;
		}
		if (dvfs->table[dvfs->step].clock > gpex_clock_get_max_clock_limit())
			dvfs->step = gpex_clock_get_table_idx(gpex_clock_get_max_clock_limit());
		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
	} else if ((dvfs->step < gpex_clock_get_table_idx(gpex_clock_get_min_clock())) &&
		   (utilization < dvfs->table[dvfs->step].min_threshold)) {
		dvfs->interactive.delay_count = 0;
		dvfs->down_requirement--;
		if (dvfs->down_requirement == 0) {
			dvfs->step++;
			dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
		}
	} else {
		dvfs->interactive.delay_count = 0;
		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
	}

	DVFS_ASSERT(dvfs->step <= gpex_clock_get_table_idx(gpex_clock_get_min_clock()));

	return 0;
}

/* --------------------------------- Joint --------------------------------- */

static int gpu_dvfs_governor_joint(int utilization)
{
	int i;
	int min_value;
	int weight_util;
	int utilT;
	int weight_fmargin_clock;
	int next_clock;
	int diff_clock;

	min_value = gpex_clock_get_min_clock();
	next_clock = gpex_clock_get_cur_clock();

	weight_util = gpu_weight_prediction_utilisation(utilization);
	utilT = ((long long)(weight_util)*gpex_clock_get_cur_clock() / 100) >> 10;
	weight_fmargin_clock =
		utilT + ((gpex_clock_get_max_clock() - utilT) / 1000) * gpex_tsg_get_freq_margin();

	if (weight_fmargin_clock > gpex_clock_get_max_clock()) {
		dvfs->step = gpex_clock_get_table_idx(gpex_clock_get_max_clock());
	} else if (weight_fmargin_clock < gpex_clock_get_min_clock()) {
		dvfs->step = gpex_clock_get_table_idx(gpex_clock_get_min_clock());
	} else {
		for (i = gpex_clock_get_table_idx(gpex_clock_get_max_clock());
		     i <= gpex_clock_get_table_idx(gpex_clock_get_min_clock()); i++) {
			diff_clock = (dvfs->table[i].clock - weight_fmargin_clock);
			if (diff_clock < min_value) {
				if (diff_clock >= 0) {
					min_value = diff_clock;
					next_clock = dvfs->table[i].clock;
				} else {
					break;
				}
			}
		}
		dvfs->step = gpex_clock_get_table_idx(next_clock);
	}

	GPU_LOG(MALI_EXYNOS_DEBUG,
		"%s: F_margin[%d] weight_util[%d] utilT[%d] weight_fmargin_clock[%d] next_clock[%d], step[%d]\n",
		__func__, gpex_tsg_get_freq_margin(), weight_util, utilT, weight_fmargin_clock,
		next_clock, dvfs->step);
	DVFS_ASSERT((dvfs->step >= gpex_clock_get_table_idx(gpex_clock_get_max_clock())) &&
		    (dvfs->step <= gpex_clock_get_table_idx(gpex_clock_get_min_clock())));

	return 0;
}

#define weight_table_size 12
#define WEIGHT_TABLE_MAX_IDX 11
int gpu_weight_prediction_utilisation(int utilization)
{
	int i;
	int idx;
	int t_window = weight_table_size;
	static int weight_sum[2] = { 0, 0 };
	static int weight_table_idx[2];
	int weight_table[WEIGHT_TABLE_MAX_IDX][weight_table_size] = {
		{ 48, 44, 40, 36, 32, 28, 24, 20, 16, 12, 8, 4 },
		{ 100, 10, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
		{ 200, 40, 8, 2, 1, 0, 0, 0, 0, 0, 0, 0 },
		{ 300, 90, 27, 8, 2, 1, 0, 0, 0, 0, 0, 0 },
		{ 400, 160, 64, 26, 10, 4, 2, 1, 0, 0, 0, 0 },
		{ 500, 250, 125, 63, 31, 16, 8, 4, 2, 1, 0, 0 },
		{ 600, 360, 216, 130, 78, 47, 28, 17, 10, 6, 4, 2 },
		{ 700, 490, 343, 240, 168, 118, 82, 58, 40, 28, 20, 14 },
		{ 800, 640, 512, 410, 328, 262, 210, 168, 134, 107, 86, 69 },
		{ 900, 810, 729, 656, 590, 531, 478, 430, 387, 349, 314, 282 },
		{ 48, 44, 40, 36, 32, 28, 24, 20, 16, 12, 8, 4 }
	};
	int normalized_util;
	int util_conv;
	int table_idx[2];

	table_idx[0] = gpex_tsg_get_weight_table_idx(0);
	table_idx[1] = gpex_tsg_get_weight_table_idx(1);

	if ((weight_table_idx[0] != table_idx[0]) || (weight_table_idx[1] != table_idx[1])) {
		weight_table_idx[0] = (table_idx[0] < WEIGHT_TABLE_MAX_IDX) ?
						    table_idx[0] :
						    WEIGHT_TABLE_MAX_IDX - 1;
		weight_table_idx[1] = (table_idx[1] < WEIGHT_TABLE_MAX_IDX) ?
						    table_idx[1] :
						    WEIGHT_TABLE_MAX_IDX - 1;
		weight_sum[0] = 0;
		weight_sum[1] = 0;
	}

	if ((weight_sum[0] == 0) || (weight_sum[1] == 0)) {
		weight_sum[0] = 0;
		weight_sum[1] = 0;
		for (i = 0; i < t_window; i++) {
			weight_sum[0] += weight_table[table_idx[0]][i];
			weight_sum[1] += weight_table[table_idx[1]][i];
		}
	}

	normalized_util = ((long long)(utilization * gpex_clock_get_cur_clock()) << 10) /
			  gpex_clock_get_max_clock();

	GPU_LOG(MALI_EXYNOS_DEBUG, "%s: util[%d] cur_clock[%d] max_clock[%d] normalized_util[%d]\n",
		__func__, utilization, gpex_clock_get_cur_clock(), gpex_clock_get_max_clock(),
		normalized_util);

	for (idx = 0; idx < 2; idx++) {
		gpex_tsg_set_weight_util(idx, 0);
		gpex_tsg_set_weight_freq(0);

		for (i = t_window - 1; i >= 0; i--) {
			if (0 == i) {
				gpex_tsg_set_util_history(idx, 0, normalized_util);
				gpex_tsg_set_weight_util(
					idx, gpex_tsg_get_weight_util(idx) +
						     gpex_tsg_get_util_history(idx, i) *
							     weight_table[table_idx[idx]][i]);
				gpex_tsg_set_weight_util(idx, gpex_tsg_get_weight_util(idx) /
								      weight_sum[idx]);
				gpex_tsg_set_en_signal(true);
				break;
			}

			gpex_tsg_set_util_history(idx, i, gpex_tsg_get_util_history(idx, i - 1));
			gpex_tsg_set_weight_util(idx,
						 gpex_tsg_get_weight_util(idx) +
							 gpex_tsg_get_util_history(idx, i) *
								 weight_table[table_idx[idx]][i]);
		}

		/* Check history */
		GPU_LOG(MALI_EXYNOS_DEBUG,
			"%s: cur_util[%d]_cur_freq[%d]_weight_util[%d]_window[%d]\n", __func__,
			utilization, gpex_clock_get_cur_clock(), gpex_tsg_get_weight_util(idx),
			t_window);
	}
	if (gpex_tsg_get_weight_util(0) < gpex_tsg_get_weight_util(1)) {
		gpex_tsg_set_weight_util(0, gpex_tsg_get_weight_util(1));
	}

	if (gpex_tsg_get_en_signal() == true)
		util_conv = (long long)(gpex_tsg_get_weight_util(0)) * gpex_clock_get_max_clock() /
			    gpex_clock_get_cur_clock();
	else
		util_conv = utilization << 10;

	return util_conv;
}

/* --------------------------------- Static -------------------------------- */

#define G3D_GOVERNOR_STATIC_PERIOD 5
#define UTIL_T1 15
#define UTIL_T2 27
#define UTIL_T3 40
#define HYSTERESIS_CYCLES 3

static int gpu_dvfs_governor_static(int utilization)
{
	static int count = 0;
	static int above_limit_cycles = 0;
	static int below_limit_cycles = 0;

	int target_freq = freq_steps[0];
	int current_freq = dvfs->table[dvfs->step].clock;
	int temp = 0;

	if (++count < G3D_GOVERNOR_STATIC_PERIOD)
		return 0;
	count = 0;

	/* Read temperature (°C) if the zone exists */
	{
		struct thermal_zone_device *tz = thermal_zone_get_zone_by_name("G3D");
		if (!IS_ERR(tz))
			thermal_zone_get_temp(tz, &temp);
		temp /= 1000;
	}

	/* Thermal clamp with hysteresis */
	if (temp >= thermal_limit_temp) {
		above_limit_cycles++;
		below_limit_cycles = 0;
		if (above_limit_cycles >= HYSTERESIS_CYCLES) {
			target_freq = freq_steps[thermal_safe_step];
			goto apply_freq;
		}
	} else {
		below_limit_cycles++;
		above_limit_cycles = 0;
	}

	/* Simple utilization bands */
	if (utilization <= UTIL_T1) {
		target_freq = freq_steps[1]; /* ~260 MHz */
	} else if (utilization <= UTIL_T2) {
		target_freq = freq_steps[4]; /* ~455 MHz */
	} else if (utilization <= UTIL_T3) {
		target_freq = freq_steps[7]; /* ~572 MHz */
	} else {
		/* Map profiles to valid top-end targets (fixing OOB indices) */
		switch (current_profile) {
		case GOV_PROFILE_SOFT:      target_freq = freq_steps[9];  break;  /* ~702 MHz */
		case GOV_PROFILE_MEDIUM:    target_freq = freq_steps[12]; break;  /* ~800 MHz */
		case GOV_PROFILE_EXTREME:   target_freq = freq_steps[13]; break;  /* ~832 MHz */
		case GOV_PROFILE_OVERCLOCK: target_freq = freq_steps[14]; break;  /* ~897 MHz */
		default:                    target_freq = freq_steps[12]; break;
		}
	}

apply_freq:
	if (current_freq != target_freq) {
		int idx = gpex_clock_get_table_idx(target_freq);
		if (idx >= 0)
			dvfs->step = idx;
	}

	return 0;
}

/* -------------------------------- Booster -------------------------------- */

static int gpu_dvfs_governor_booster(int utilization)
{
	static int weight;
	int cur_weight, booster_threshold, dvfs_table_lock;

	cur_weight = gpex_clock_get_cur_clock() * utilization;
	/* booster_threshold = current clock * set the percentage of utilization */
	booster_threshold = gpex_clock_get_cur_clock() * 50;

	dvfs_table_lock = gpex_clock_get_table_idx(gpex_clock_get_max_clock());

	if ((dvfs->step >= dvfs_table_lock + 2) && ((cur_weight - weight) > booster_threshold)) {
		dvfs->step -= 2;
		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
		GPU_LOG(MALI_EXYNOS_WARNING, "Booster Governor: G3D level 2 step\n");
	} else if ((dvfs->step > gpex_clock_get_table_idx(gpex_clock_get_max_clock())) &&
		   (utilization > dvfs->table[dvfs->step].max_threshold)) {
		dvfs->step--;
		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
	} else if ((dvfs->step < gpex_clock_get_table_idx(gpex_clock_get_min_clock())) &&
		   (utilization < dvfs->table[dvfs->step].min_threshold)) {
		dvfs->down_requirement--;
		if (dvfs->down_requirement == 0) {
			dvfs->step++;
			dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
		}
	} else {
		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
	}

	DVFS_ASSERT((dvfs->step >= gpex_clock_get_table_idx(gpex_clock_get_max_clock())) &&
		    (dvfs->step <= gpex_clock_get_table_idx(gpex_clock_get_min_clock())));

	weight = cur_weight;

	return 0;
}

/* -------------------------------- Dynamic -------------------------------- */

static int gpu_dvfs_governor_dynamic(int utilization)
{
	int max_clock_lev = gpex_clock_get_table_idx(gpex_clock_get_max_clock());
	int min_clock_lev = gpex_clock_get_table_idx(gpex_clock_get_min_clock());

	if ((dvfs->step > max_clock_lev) && (utilization > dvfs->table[dvfs->step].max_threshold)) {
		if (dvfs->table[dvfs->step].clock * utilization >
		    dvfs->table[dvfs->step - 1].clock * dvfs->table[dvfs->step - 1].max_threshold) {
			dvfs->step -= 2;
			if (dvfs->step < max_clock_lev) {
				dvfs->step = max_clock_lev;
			}
		} else {
			dvfs->step--;
		}

		if (dvfs->table[dvfs->step].clock > gpex_clock_get_max_clock_limit())
			dvfs->step = gpex_clock_get_table_idx(gpex_clock_get_max_clock_limit());

		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
	} else if ((dvfs->step < min_clock_lev) &&
		   (utilization < dvfs->table[dvfs->step].min_threshold)) {
		dvfs->down_requirement--;
		if (dvfs->down_requirement == 0) {
			if (dvfs->table[dvfs->step].clock * utilization <
			    dvfs->table[dvfs->step + 1].clock *
				    dvfs->table[dvfs->step + 1].min_threshold) {
				dvfs->step += 2;
				if (dvfs->step > min_clock_lev) {
					dvfs->step = min_clock_lev;
				}
			} else {
				dvfs->step++;
			}
			dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
		}
	} else {
		dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
	}

	DVFS_ASSERT(dvfs->step <= gpex_clock_get_table_idx(gpex_clock_get_min_clock()));

	return 0;
}

/* ----------------------------- Performance ------------------------------- */

static int gpu_dvfs_governor_performance(int utilization)
{
	int max_allowed_idx = gpex_clock_get_table_idx(gpex_clock_get_max_clock_limit());

	if (max_allowed_idx < 0)
		max_allowed_idx = gpex_clock_get_table_idx(gpex_clock_get_max_clock());

	dvfs->step = max_allowed_idx;
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                    Energy_ondemand: private state & sysfs                 */
/* ------------------------------------------------------------------------- */

struct energy_ondemand_state {
	/* Core policy */
	int target_util_pct;          /* PI setpoint (0..100) */
	int util_high_pct;            /* hard boost threshold */
	int util_low_pct;             /* gentle downshift threshold */

	/* PI controller gains (Q8.8 fixed point for stability) */
	int kp_q8;                    /* proportional gain */
	int ki_q8;                    /* integral gain */
	int integrator_q8;            /* accumulated error */

	/* Step control & damping */
	int up_step_jump;             /* how many levels to jump up on heavy load */
	int down_step_max;            /* at most this many steps down in a decision */
	int min_dwell_up_ms;          /* min time before next up move */
	int min_dwell_down_ms;        /* min time before next down move */
	int staycount_bonus;          /* extra staycount while cooling */

	/* Thermal management */
	int thermal_headroom_degC;    /* start soft-throttling when within this headroom */
	int thermal_cool_bias;        /* bias (levels) to push down when hot */
	int thermal_emergency_degC;   /* immediate clamp temperature */
	int safe_step_index;          /* step used as safe floor in emergencies */

	/* Frame spike heuristic */
	int fps_boost_enable;         /* enable utilization-spike guided boosting */
	int frame_time_target_us;     /* reserved for future (FPS-aware) */
	int frame_spike_sensitivity;  /* spikes required to trigger jump */

	/* Hysteresis */
	int hysteresis_cycles_up;
	int hysteresis_cycles_down;
	int up_counter;
	int down_counter;

	/* Derived/Runtime */
	ktime_t last_up_time;
	ktime_t last_down_time;

	/* Moved former statics here */
	int prev_weight;
	int spike_ctr;
};

static struct energy_ondemand_state eod = {
	/* Balanced defaults (tunable via sysfs) */
	.target_util_pct        = 80,
	.util_high_pct          = 72,
	.util_low_pct           = 28,

	.kp_q8                  = 140, /* ~0.55 */
	.ki_q8                  = 16,  /* ~0.06 */

	.up_step_jump           = 2,
	.down_step_max          = 1,
	.min_dwell_up_ms        = 8,
	.min_dwell_down_ms      = 24,
	.staycount_bonus        = 1,

	.thermal_headroom_degC  = 6,
	.thermal_cool_bias      = 1,
	.thermal_emergency_degC = 82,
	.safe_step_index        = 12, /* ~800 MHz */

	.fps_boost_enable       = 1,
	.frame_time_target_us   = 16666, /* 60 fps */
	.frame_spike_sensitivity= 2,

	.hysteresis_cycles_up   = 1,
	.hysteresis_cycles_down = 3,
	.up_counter             = 0,
	.down_counter           = 0,

	.last_up_time           = 0,
	.last_down_time         = 0,

	.prev_weight            = 0,
	.spike_ctr              = 0,
};

/* Helpers */
static inline int clamp_int(int v, int lo, int hi)
{
	return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline bool eod_dwell_expired(ktime_t last, int need_ms)
{
	if (need_ms <= 0) return true;
	return ktime_to_ms(ktime_sub(ktime_get(), last)) >= need_ms;
}

static inline int eod_level_of_clock(int hz)
{
	int idx = gpex_clock_get_table_idx(hz);
	if (idx < 0) idx = gpex_clock_get_table_idx(gpex_clock_get_min_clock());
	return idx;
}

static int eod_compute_effort(int target_pct, int actual_pct)
{
	int err = target_pct - actual_pct;
	/* Integrator with light anti-windup */
	eod.integrator_q8 = clamp_int(eod.integrator_q8 + (err << 8), -(1<<15), (1<<15));
	return eod.kp_q8 * err + ((eod.ki_q8 * eod.integrator_q8) >> 8);
}

static void eod_sanitize_params(void)
{
	eod.target_util_pct  = clamp_int(eod.target_util_pct, 10, 99);
	eod.util_high_pct    = clamp_int(eod.util_high_pct,  20, 99);
	eod.util_low_pct     = clamp_int(eod.util_low_pct,   1,  60);
	eod.up_step_jump     = clamp_int(eod.up_step_jump,   1,  4);
	eod.down_step_max    = clamp_int(eod.down_step_max,  1,  3);
	eod.safe_step_index  = clamp_int(eod.safe_step_index, 0, num_steps-1);
	eod.kp_q8            = clamp_int(eod.kp_q8,           0,  512);
	eod.ki_q8            = clamp_int(eod.ki_q8,           0,  256);
}

/* sysfs: expose primary tunables (simple key=value parser) */
static ssize_t eod_params_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE,
		"target_util=%d util_high=%d util_low=%d kp_q8=%d ki_q8=%d up_jump=%d down_max=%d "
		"dwell_up_ms=%d dwell_down_ms=%d therm_headroom=%d therm_cool_bias=%d "
		"therm_emerg=%d safe_step=%d fps_boost=%d ft_target_us=%d spike_sense=%d\n",
		eod.target_util_pct, eod.util_high_pct, eod.util_low_pct, eod.kp_q8, eod.ki_q8,
		eod.up_step_jump, eod.down_step_max, eod.min_dwell_up_ms, eod.min_dwell_down_ms,
		eod.thermal_headroom_degC, eod.thermal_cool_bias, eod.thermal_emergency_degC,
		eod.safe_step_index, eod.fps_boost_enable, eod.frame_time_target_us,
		eod.frame_spike_sensitivity);
}

static ssize_t eod_params_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	/* Format: key=value pairs separated by spaces/newlines */
	int v;
#define UPD_INT(key, field) if (sscanf(p, key "=%d", &v) == 1) eod.field = v
	const char *p = buf;

	while (*p) {
		UPD_INT("target_util", target_util_pct);
		UPD_INT("util_high", util_high_pct);
		UPD_INT("util_low",  util_low_pct);
		UPD_INT("kp_q8",     kp_q8);
		UPD_INT("ki_q8",     ki_q8);
		UPD_INT("up_jump",   up_step_jump);
		UPD_INT("down_max",  down_step_max);
		UPD_INT("dwell_up_ms",   min_dwell_up_ms);
		UPD_INT("dwell_down_ms", min_dwell_down_ms);
		UPD_INT("therm_headroom", thermal_headroom_degC);
		UPD_INT("therm_cool_bias", thermal_cool_bias);
		UPD_INT("therm_emerg", thermal_emergency_degC);
		UPD_INT("safe_step",  safe_step_index);
		UPD_INT("fps_boost",  fps_boost_enable);
		UPD_INT("ft_target_us", frame_time_target_us);
		UPD_INT("spike_sense", frame_spike_sensitivity);

		while (*p && *p != ' ' && *p != '\n' && *p != '\t') p++;
		while (*p == ' ' || *p == '\n' || *p == '\t') p++;
	}

	eod_sanitize_params();
	return count;
}
static DEVICE_ATTR_RW(eod_params);

/* Create eod sysfs node (fixed: no recursion / correct error returns) */
static int energy_ondemand_sysfs_init(void)
{
	struct device *dev = gpu_get_device();
	int ret = device_create_file(dev, &dev_attr_eod_params);
	if (ret) {
		GPU_LOG(MALI_EXYNOS_WARNING,
			"Energy_ondemand: failed to create sysfs params (%d)\n", ret);
	}
	return ret;
}

/* ------------------------------------------------------------------------- */
/*                       Energy_ondemand governor core                        */
/* ------------------------------------------------------------------------- */

static int gpu_dvfs_governor_energy_ondemand(int utilization)
{
	/* Safety rails and locals */
	int max_idx = gpex_clock_get_table_idx(gpex_clock_get_max_clock());
	int min_idx = gpex_clock_get_table_idx(gpex_clock_get_min_clock());
	int cur_idx = dvfs->step;
	int temp_mc = 0;   /* milliC */

	/* Temperature (may fail if zone missing) */
	{
		struct thermal_zone_device *tz = thermal_zone_get_zone_by_name("G3D");
		if (!IS_ERR(tz))
			thermal_zone_get_temp(tz, &temp_mc);
	}
	const int temp_c = temp_mc / 1000;

	/* Emergency clamp: immediate move to safe level */
	if (temp_c >= eod.thermal_emergency_degC) {
		int safe_idx = clamp_int(eod.safe_step_index, max_idx, min_idx);
		if (cur_idx < safe_idx) {
			if (eod_dwell_expired(eod.last_down_time, eod.min_dwell_down_ms)) {
				dvfs->step = safe_idx;
				eod.last_down_time = ktime_get();
			}
		} else {
			dvfs->step = safe_idx;
		}
		/* Keep down_requirement sticky while overheated */
		dvfs->down_requirement =
			dvfs->table[dvfs->step].down_staycount + eod.staycount_bonus;
		goto out_assert;
	}

	/* Use predicted util to smooth PI (Q10 -> pct) */
	{
		int util_q10 = gpu_weight_prediction_utilisation(utilization);
		int util_pred_pct = clamp_int((util_q10 >> 10), 0, 100);
		/* Mix: 70% predicted, 30% instant */
		utilization = (util_pred_pct * 7 + utilization * 3) / 10;
	}

	/* Spike-based fast boost (heuristic) */
	if (eod.fps_boost_enable) {
		if (utilization >= eod.util_high_pct) {
			eod.spike_ctr++;
			if (eod.spike_ctr >= eod.frame_spike_sensitivity &&
			    eod_dwell_expired(eod.last_up_time, eod.min_dwell_up_ms)) {
				int jump = clamp_int(eod.up_step_jump, 1, 4);
				dvfs->step = max(cur_idx - jump, max_idx);
				eod.last_up_time = ktime_get();
				eod.spike_ctr = 0;
				dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
				GPU_LOG(MALI_EXYNOS_WARNING,
					"Energy_ondemand: spike boost (%d steps)\n", jump);
				goto out_assert;
			}
		} else if (eod.spike_ctr > 0) {
			eod.spike_ctr--;
		}
	}

	/* Sudden-load detector (derivative-like) */
	{
		int weight = gpex_clock_get_cur_clock() * utilization;
		int boost_thr = gpex_clock_get_cur_clock() *
				((eod.util_high_pct > 40) ? eod.util_high_pct : 40);
		if ((cur_idx > max_idx) && ((weight - eod.prev_weight) > boost_thr) &&
		    eod_dwell_expired(eod.last_up_time, eod.min_dwell_up_ms)) {
			int jump = clamp_int(eod.up_step_jump, 1, 4);
			dvfs->step = max(cur_idx - jump, max_idx);
			eod.last_up_time = ktime_get();
			dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
			GPU_LOG(MALI_EXYNOS_WARNING, "Energy_ondemand: sudden boost (%d)\n", jump);
			eod.prev_weight = weight;
			goto out_assert;
		}
		eod.prev_weight = weight;
	}

	/* PI control -> step decision */
	{
		int effort_q8 = eod_compute_effort(eod.target_util_pct, utilization);
		int delta_steps = effort_q8 >> 8; /* back to int domain */

		/* Thermal soft bias when near limit: nudge downward */
		if (thermal_limit_temp > 0 &&
		    temp_c >= (thermal_limit_temp - eod.thermal_headroom_degC)) {
			delta_steps -= eod.thermal_cool_bias;
		}

		if (delta_steps < 0) {
			/* Increase freq (lower index) */
			if (eod_dwell_expired(eod.last_up_time, eod.min_dwell_up_ms)) {
				int steps = clamp_int(-delta_steps, 1, eod.up_step_jump);
				int tgt = max(cur_idx - steps, max_idx);
				if (++eod.up_counter >= eod.hysteresis_cycles_up) {
					dvfs->step = tgt;
					eod.last_up_time = ktime_get();
					eod.up_counter = 0;
					dvfs->down_requirement =
						dvfs->table[dvfs->step].down_staycount;
				}
			}
		} else if (delta_steps > 0) {
			/* Decrease freq (higher index) slowly */
			if (eod_dwell_expired(eod.last_down_time, eod.min_dwell_down_ms)) {
				int steps = clamp_int(delta_steps, 1, eod.down_step_max);
				int tgt = min(cur_idx + steps, min_idx);
				if (++eod.down_counter >= eod.hysteresis_cycles_down) {
					dvfs->step = tgt;
					eod.last_down_time = ktime_get();
					eod.down_counter = 0;
					dvfs->down_requirement =
						dvfs->table[dvfs->step].down_staycount +
						eod.staycount_bonus;
				} else {
					dvfs->down_requirement =
						dvfs->table[dvfs->step].down_staycount + 1;
				}
			} else {
				dvfs->down_requirement =
					dvfs->table[dvfs->step].down_staycount + 1;
			}
		} else {
			/* Neutral: maintain staycount */
			dvfs->down_requirement = dvfs->table[dvfs->step].down_staycount;
		}
	}

	/* Honor CLBoost if active */
	if (gpex_clboost_check_activation_condition()) {
		dvfs->step = max_idx;
	}

	/* Hard limits */
	if (dvfs->table[dvfs->step].clock > gpex_clock_get_max_clock_limit())
		dvfs->step = eod_level_of_clock(gpex_clock_get_max_clock_limit());

out_assert:
	DVFS_ASSERT((dvfs->step >= max_idx) && (dvfs->step <= min_idx));
	return 0;
}

/* ------------------------------------------------------------------------- */
/*                                Init / Hooks                               */
/* ------------------------------------------------------------------------- */

static int __init gpu_governor_sysfs_init(void)
{
	int ret;
	struct device *dev = gpu_get_device();

	ret = device_create_file(dev, &dev_attr_profile);
	if (ret)
		return ret;

	ret = device_create_file(dev, &dev_attr_thermal_limit);
	if (ret)
		device_remove_file(dev, &dev_attr_profile);

	/* Energy_ondemand tunables */
	if (!ret) {
		int r2 = energy_ondemand_sysfs_init();
		if (r2)
			GPU_LOG(MALI_EXYNOS_WARNING,
				"Energy_ondemand: failed to create sysfs params\n");
	}
	return ret;
}

int gpu_dvfs_decide_next_freq(int utilization)
{
	unsigned long flags;

	if (gpex_tsg_get_migov_mode() == 1 && gpex_tsg_get_is_gov_set() != 1) {
		gpu_dvfs_governor_setting_locked(G3D_DVFS_GOVERNOR_JOINT);
		gpex_tsg_set_saved_polling_speed(gpex_dvfs_get_polling_speed());
		gpex_dvfs_set_polling_speed(16);
		gpex_tsg_set_is_gov_set(1);
		gpex_tsg_set_en_signal(false);
		gpex_tsg_set_pmqos(true);
	} else if (gpex_tsg_get_migov_mode() != 1 && gpex_tsg_get_is_gov_set() != 0) {
		gpu_dvfs_governor_setting_locked(gpex_tsg_get_governor_type_init());
		gpex_dvfs_set_polling_speed(gpex_tsg_get_saved_polling_speed());
		gpex_tsg_set_is_gov_set(0);
		gpex_tsg_set_pmqos(false);
		gpex_tsg_reset_acc_count();
	}

	gpex_tsg_input_nr_acc_cnt();

	spin_lock_irqsave(&dvfs->spinlock, flags);
	gpu_dvfs_get_next_level(utilization);
	spin_unlock_irqrestore(&dvfs->spinlock, flags);

	if (gpex_clboost_check_activation_condition())
		dvfs->step = gpex_clock_get_table_idx(gpex_clock_get_max_clock());

	return dvfs->table[dvfs->step].clock;
}

int gpu_dvfs_governor_setting(int governor_type)
{
	unsigned long flags;

	if ((governor_type < 0) || (governor_type >= G3D_MAX_GOVERNOR_NUM)) {
		GPU_LOG(MALI_EXYNOS_WARNING, "%s: invalid governor type (%d)\n", __func__,
			governor_type);
		return -1;
	}

	spin_lock_irqsave(&dvfs->spinlock, flags);
	dvfs->step = gpex_clock_get_table_idx(governor_info[governor_type].start_clk);
	gpu_dvfs_get_next_level = (GET_NEXT_LEVEL)(governor_info[governor_type].governor);

	dvfs->env_data.utilization = 80;

	dvfs->down_requirement = 1;
	dvfs->governor_type = governor_type;

	spin_unlock_irqrestore(&dvfs->spinlock, flags);

	return 0;
}

int gpu_dvfs_governor_setting_locked(int governor_type)
{
	unsigned long flags;

	if ((governor_type < 0) || (governor_type >= G3D_MAX_GOVERNOR_NUM)) {
		GPU_LOG(MALI_EXYNOS_WARNING, "%s: invalid governor type (%d)\n", __func__,
			governor_type);
		return -1;
	}

	spin_lock_irqsave(&dvfs->spinlock, flags);
	dvfs->step = gpex_clock_get_table_idx(governor_info[governor_type].start_clk);
	gpu_dvfs_get_next_level = (GET_NEXT_LEVEL)(governor_info[governor_type].governor);

	dvfs->governor_type = governor_type;

	spin_unlock_irqrestore(&dvfs->spinlock, flags);

	return 0;
}

int gpu_dvfs_governor_init(struct dvfs_info *_dvfs)
{
	int governor_type = G3D_DVFS_GOVERNOR_ENERGY_ONDEMAND;

	dvfs = _dvfs;

	governor_type = dvfs->governor_type;

	if (gpu_dvfs_governor_setting(governor_type) < 0) {
		GPU_LOG(MALI_EXYNOS_WARNING, "%s: fail to initialize governor\n", __func__);
		return -1;
	}

	/* Make sure tunables are in a sane range if userspace wrote bad values before */
	eod_sanitize_params();
	return 0;
}
