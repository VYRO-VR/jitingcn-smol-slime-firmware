/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "globals.h"
#include "system/system.h"
#include "system/watchdog.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "cal_sample.h"
#include "cal_sens.h"
#include "util.h"

#if CONFIG_SENSOR_USE_SENS_CALIBRATION

LOG_MODULE_REGISTER(cal_sens, LOG_LEVEL_INF);

/* Latched by sensor_request_calibration_sens() in calibration.c */
extern uint8_t sens_cal_axis;
extern uint16_t sens_cal_revolutions;

/* Published progress/outcome for the host (uplink sub-packet type 6).
 *
 * The calibration runs on the calibration thread while the connection thread
 * reads, so the report is packed into two atomic words rather than a struct.
 * The detail word is always published before the report word: a reader that
 * takes the report word first and the detail word second can therefore never
 * pair a terminal phase with a scale from the previous run.
 */
static atomic_t sens_cal_report_word;  /* phase | result << 8 | axis << 16 | seq << 24 */
static atomic_t sens_cal_detail_word;  /* scale_q12 | progress << 16 */
static uint8_t sens_cal_seq;           /* Calibration thread only */
static uint8_t sens_cal_report_axis;   /* Calibration thread only */
static uint16_t sens_cal_report_deg;   /* Calibration thread only */

static void sens_cal_publish_detail(uint16_t scale_q12)
{
	atomic_set(&sens_cal_detail_word, (atomic_val_t)((uint32_t)scale_q12 | ((uint32_t)sens_cal_report_deg << 16)));
}

static void sens_cal_publish(uint8_t phase, uint8_t result, uint16_t scale_q12)
{
	if (phase == SENS_CAL_PHASE_DONE) {
		sens_cal_seq++;
	}
	sens_cal_publish_detail(scale_q12);
	atomic_set(
		&sens_cal_report_word,
		(atomic_val_t)((uint32_t)phase | ((uint32_t)result << 8) | ((uint32_t)sens_cal_report_axis << 16)
					   | ((uint32_t)sens_cal_seq << 24))
	);
}

/* Fixed-point scale for the wire, saturating. Rejected runs still report the
 * computed value so the host can show why it was rejected. */
static uint16_t sens_cal_scale_q12(float scale)
{
	if (!(scale > 0.0f)) { /* Also catches NaN */
		return 0;
	}
	float q = scale * (float)(1 << SENS_CAL_SCALE_Q12_SHIFT);
	if (q > 65535.0f) {
		return 65535;
	}
	return (uint16_t)(q + 0.5f);
}

void sens_cal_get_report(struct sens_cal_report *out)
{
	uint32_t report = (uint32_t)atomic_get(&sens_cal_report_word);
	uint32_t detail = (uint32_t)atomic_get(&sens_cal_detail_word);

	out->phase = report & 0xFF;
	out->result = (report >> 8) & 0xFF;
	out->axis = (report >> 16) & 0xFF;
	out->seq = (report >> 24) & 0xFF;
	out->scale_q12 = detail & 0xFFFF;
	out->progress = (detail >> 16) & 0xFFFF;
}

// =============================================================================
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
// The user spins the tracker a known number of full revolutions about a single
// axis. We integrate the measured gyro rate over that motion and compare the
// measured angle against the true angle to derive a per-axis scale factor that
// corrects cumulative over- or under-rotation.
#define SENS_CAL_BIAS_SAMPLE_MS 1000    // In-situ bias averaging window
#define SENS_CAL_START_RATE_DPS 30.0f   // Rate that counts as "spin started"
#define SENS_CAL_STOP_RATE_DPS 10.0f    // Rate that counts as "spin stopped"
#define SENS_CAL_STOP_DWELL_MS 1000     // Rate must stay low this long to stop
#define SENS_CAL_START_TIMEOUT_MS 30000 // Give up waiting for the spin to start
#define SENS_CAL_SPIN_TIMEOUT_MS 60000  // Give up waiting for the spin to finish
#define SENS_CAL_MIN_FRACTION 0.85f     // Require near-complete expected angle before stopping
#define SENS_CAL_MIN_SCALE 0.9f         // Reject implausible results (likely wrong turn count)
#define SENS_CAL_MAX_SCALE 1.1f
#define SENS_CAL_WARN_OFF_AXIS_RATIO 0.10f
#define SENS_CAL_MAX_OFF_AXIS_RATIO 0.25f

void sensor_calibrate_sens(void)
{
	uint8_t axis = sens_cal_axis;
	uint16_t revolutions = sens_cal_revolutions;

	sens_cal_report_axis = axis > 2 ? 0 : axis;
	sens_cal_report_deg = 0;

	if (axis > 2 || revolutions == 0) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_INVALID_PARAMS, 0);
		LOG_ERR("Sensitivity calibration: invalid parameters");
		printk("Gyro sensitivity auto-calibration failed: invalid parameters.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}
	char axis_char = "XYZ"[axis];
	float expected_deg = 360.0f * revolutions;

	LOG_INF(
		"Sensitivity calibration: axis %c, %u rev (%.1f deg expected)",
		axis_char,
		revolutions,
		(double)expected_deg
	);

	float g[3];

	// 1. Wait for the tracker to be held still before measuring bias.
	set_led(SYS_LED_PATTERN_LONG, SYS_LED_PRIORITY_SENSOR);
	LOG_INF("Sensitivity calibration: hold still");
	sens_cal_publish(SENS_CAL_PHASE_HOLD_STILL, SENS_CAL_RESULT_NONE, 0);
	if (!wait_for_motion(false, 6)) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NOT_STILL, 0);
		LOG_WRN("Sensitivity calibration: tracker not still, aborting");
		printk("Gyro sensitivity auto-calibration failed: tracker was not still.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	// 2. Measure the in-situ gyro bias. sensor_wait_gyro returns
	//    raw samples (before bias and sensitivity are applied), so we average a
	//    short window here rather than relying on the stored gyro bias.
	sens_cal_publish(SENS_CAL_PHASE_BIAS, SENS_CAL_RESULT_NONE, 0);
	double bias_sum[3] = {0.0, 0.0, 0.0};
	int bias_count = 0;
	int64_t bias_start = k_uptime_get();
	while (k_uptime_get() - bias_start < SENS_CAL_BIAS_SAMPLE_MS) {
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_GYRO_TIMEOUT, 0);
			LOG_WRN("Sensitivity calibration: gyro timeout during bias, aborting");
			printk("Gyro sensitivity auto-calibration failed: gyro timeout while measuring bias.\n");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;
		}
		for (int i = 0; i < 3; i++) {
			bias_sum[i] += (double)g[i];
		}
		bias_count++;
		watchdog_feed(WDT_CHANNEL_CALIBRATION);
	}
	if (bias_count == 0) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NO_BIAS_SAMPLES, 0);
		LOG_WRN("Sensitivity calibration: no bias samples, aborting");
		printk("Gyro sensitivity auto-calibration failed: no bias samples.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}
	float gyro_bias[3];
	for (int i = 0; i < 3; i++) {
		gyro_bias[i] = (float)(bias_sum[i] / bias_count);
	}
	LOG_INF(
		"Sensitivity calibration: bias %.4f %.4f %.4f dps",
		(double)gyro_bias[0],
		(double)gyro_bias[1],
		(double)gyro_bias[2]
	);

	// 3. Arm and wait for the user to start spinning. FLASH means "ready, spin now".
	set_led(SYS_LED_PATTERN_FLASH, SYS_LED_PRIORITY_SENSOR);
	LOG_INF("Sensitivity calibration: spin the tracker about the %c axis now", axis_char);
	sens_cal_publish(SENS_CAL_PHASE_ARMED, SENS_CAL_RESULT_NONE, 0);
	int64_t arm_start = k_uptime_get();
	int64_t last_wdt = arm_start;
	float rate = 0.0f;
	while (true) {
		if (k_uptime_get() - arm_start >= SENS_CAL_START_TIMEOUT_MS) {
			sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NO_SPIN, 0);
			LOG_WRN("Sensitivity calibration: no spin detected, aborting");
			printk("Gyro sensitivity auto-calibration failed: no spin detected.\n");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;
		}
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			continue; // Tolerate occasional waits while watching for the start
		}
		rate = g[axis] - gyro_bias[axis];
		if (k_uptime_get() - last_wdt >= 1000) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
			last_wdt = k_uptime_get();
		}
		if (fabsf(rate) > SENS_CAL_START_RATE_DPS) {
			break;
		}
	}

	// 4. Integrate the gyro rate over the spin. ON indicates recording.
	//    sensor_wait_gyro returns only the most recent sample, so crediting each
	//    observed sample a fixed 1/ODR step would silently drop the rotation from
	//    any samples produced while this loop was busy and undercount the spin.
	//    Integrate against the real elapsed time between samples instead (as the
	//    fusion path does with its measured time step); each observed sample then
	//    covers the true interval since the previous one, which also tolerates the
	//    sensor's actual sample rate differing from its nominal ODR.
	set_led(SYS_LED_PATTERN_ON, SYS_LED_PRIORITY_SENSOR);
	LOG_INF("Sensitivity calibration: recording");
	sens_cal_publish(SENS_CAL_PHASE_RECORDING, SENS_CAL_RESULT_NONE, 0);
	double measured = 0.0;
	double axis_motion = 0.0;
	double off_axis_motion = 0.0;
	int64_t spin_start = k_uptime_get();
	int64_t last_ticks = k_uptime_ticks();
	int64_t below_since = -1; // When the rate first dropped below the stop threshold
	int64_t last_progress = 0;
	bool finished = false;
	last_wdt = spin_start;
	while (k_uptime_get() - spin_start < SENS_CAL_SPIN_TIMEOUT_MS) {
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_GYRO_TIMEOUT, 0);
			LOG_WRN("Sensitivity calibration: gyro timeout during spin, aborting");
			printk("Gyro sensitivity auto-calibration failed: gyro timeout during spin.\n");
			set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
			return;
		}
		int64_t now_ticks = k_uptime_ticks();
		double dt = (double)k_ticks_to_us_near64(now_ticks - last_ticks) * 1e-6;
		last_ticks = now_ticks;

		rate = g[axis] - gyro_bias[axis];
		measured += (double)rate * dt;
		axis_motion += fabs((double)rate) * dt;
		double off_axis_rate_sq = 0.0;
		for (int i = 0; i < 3; i++) {
			if (i != axis) {
				double off_rate = (double)(g[i] - gyro_bias[i]);
				off_axis_rate_sq += off_rate * off_rate;
			}
		}
		off_axis_motion += sqrt(off_axis_rate_sq) * dt;

		if (k_uptime_get() - last_wdt >= 1000) {
			watchdog_feed(WDT_CHANNEL_CALIBRATION);
			last_wdt = k_uptime_get();
		}

		// Publish the accumulated angle a few times a second so the host can drive
		// a live turn counter. Rate limited because this loop runs at gyro ODR.
		if (k_uptime_get() - last_progress >= 100) {
			last_progress = k_uptime_get();
			double deg = fabs(measured);
			sens_cal_report_deg = deg >= 65535.0 ? 65535 : (uint16_t)deg;
			sens_cal_publish_detail(0);
		}

		// The spin is complete once the rate stays low for the dwell time, but only
		// after at least a minimum fraction of the expected angle has been covered.
		// This keeps a brief pause mid-spin from ending the measurement early.
		if (fabsf(rate) < SENS_CAL_STOP_RATE_DPS && fabs(measured) >= (double)(expected_deg * SENS_CAL_MIN_FRACTION)) {
			if (below_since < 0) {
				below_since = k_uptime_get();
			} else if (k_uptime_get() - below_since >= SENS_CAL_STOP_DWELL_MS) {
				finished = true;
				break;
			}
		} else {
			below_since = -1;
		}
	}

	if (!finished) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_SPIN_TIMEOUT, 0);
		LOG_WRN("Sensitivity calibration: spin did not complete in time, aborting");
		printk("Gyro sensitivity auto-calibration failed: spin did not complete in time.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	float measured_deg = (float)fabs(measured);
	sens_cal_report_deg = measured_deg >= 65535.0f ? 65535 : (uint16_t)measured_deg;
	if (measured_deg < 1e-3f) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_ANGLE_TOO_SMALL, 0);
		LOG_WRN("Sensitivity calibration: measured angle too small, aborting");
		printk("Gyro sensitivity auto-calibration failed: measured angle too small.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	// Want measured_deg * scale == expected_deg, independent of rotation direction.
	float scale = expected_deg / measured_deg;
	float over_rotation = measured_deg - expected_deg;
	float off_axis_ratio = axis_motion > 1e-3 ? (float)(off_axis_motion / axis_motion) : 1.0f;
	float equivalent_diff_deg = (1.0f - (1.0f / scale)) * (360.0f * CONFIG_SENSOR_SENS_REV);
	LOG_INF(
		"Sensitivity calibration: measured %.2f deg, expected %.2f deg, over-rotation %.2f deg, off-axis %.3f",
		(double)measured_deg,
		(double)expected_deg,
		(double)over_rotation,
		(double)off_axis_ratio
	);
	LOG_INF("Sensitivity calibration: computed scale %.5f", (double)scale);

	if (!v_finite(&scale, 1)) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_INVALID_SCALE, 0);
		LOG_WRN("Sensitivity calibration: computed non-finite scale, not applied");
		printk("Gyro sensitivity auto-calibration rejected: invalid scale. Nothing saved.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	if (off_axis_ratio > SENS_CAL_MAX_OFF_AXIS_RATIO) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_OFF_AXIS, sens_cal_scale_q12(scale));
		LOG_WRN(
			"Sensitivity calibration: off-axis ratio %.3f above %.3f, not applied",
			(double)off_axis_ratio,
			(double)SENS_CAL_MAX_OFF_AXIS_RATIO
		);
		printk(
			"Gyro sensitivity auto-calibration rejected: too much off-axis motion (%.2f > %.2f). Nothing saved.\n",
			(double)off_axis_ratio,
			(double)SENS_CAL_MAX_OFF_AXIS_RATIO
		);
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	// Reject implausible results. The firmware cannot distinguish a wrong revolution
	// count from a genuinely large sensitivity error, so a scale far from 1.0 most
	// likely means the wrong number of turns was performed.
	if (scale < SENS_CAL_MIN_SCALE || scale > SENS_CAL_MAX_SCALE) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_SCALE_RANGE, sens_cal_scale_q12(scale));
		LOG_WRN(
			"Sensitivity calibration: scale %.5f out of range [%.2f, %.2f], not applied",
			(double)scale,
			(double)SENS_CAL_MIN_SCALE,
			(double)SENS_CAL_MAX_SCALE
		);
		printk(
			"Gyro sensitivity auto-calibration rejected: measured %.2f deg for %.2f deg, scale %.5f outside "
			"%.2f..%.2f. Equivalent sens diff over %u rev: %.3f deg. Nothing saved.\n",
			(double)measured_deg,
			(double)expected_deg,
			(double)scale,
			(double)SENS_CAL_MIN_SCALE,
			(double)SENS_CAL_MAX_SCALE,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg
		);
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	if (!retained) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NO_RETAINED, sens_cal_scale_q12(scale));
		LOG_ERR("Sensitivity calibration: retained data unavailable, not applied");
		printk("Gyro sensitivity auto-calibration failed: retained data unavailable.\n");
		set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_SENSOR);
		return;
	}

	retained->gyroSensScale[axis] = scale;
	retained_update();
	sys_write(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, retained->gyroSensScale, sizeof(retained->gyroSensScale));

	sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_OK, sens_cal_scale_q12(scale));
	LOG_INF("Sensitivity calibration: axis %c scale set to %.5f", axis_char, (double)scale);
	if (off_axis_ratio > SENS_CAL_WARN_OFF_AXIS_RATIO) {
		printk(
			"Gyro sensitivity auto-calibration saved: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, "
			"off-axis %.2f. Axis alignment was loose; repeating may improve accuracy.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	} else {
		printk(
			"Gyro sensitivity auto-calibration saved: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, "
			"off-axis %.2f.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	}
	set_led(SYS_LED_PATTERN_ONESHOT_COMPLETE, SYS_LED_PRIORITY_SENSOR);
}
#endif

#endif /* CONFIG_SENSOR_USE_SENS_CALIBRATION */
