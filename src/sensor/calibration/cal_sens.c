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

#include <errno.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>

#include "cal_sample.h"
#include "cal_sens.h"
#include "calibration.h"
#include "util.h"
#include "connection/tracker_events.h"
#if CONFIG_SENSOR_TCAL_HEATED
#include "tcal_heated.h"
#endif

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
/* Serialize explicit replacements with the measurement's final application. */
K_MUTEX_DEFINE(sensitivity_lock);
static uint32_t sensitivity_generation;
static uint16_t sensitivity_operation;
static struct led_token sensitivity_feedback;

static void sensitivity_replace_locked(uint8_t reason)
{
	cal_event_end(sensitivity_operation, CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLIED, reason);
	sensor_calibration_result(sensitivity_feedback, LED_CANCELLED);
	sensitivity_feedback = (struct led_token){0};
	sensitivity_operation = 0;
	sensitivity_generation++;
}

static void sensitivity_step(uint16_t operation_id, struct led_token feedback, uint8_t phase, uint8_t detail)
{
	cal_event_step(operation_id, phase, detail);
	sensor_calibration_stage(feedback, phase == CAL_PHASE_COLLECT ? LED_COLLECT_STILL :
		phase == CAL_PHASE_WAIT_ROTATION ? LED_WAIT_MOVE :
		phase == CAL_PHASE_RECORD_ROTATION ? LED_COLLECT_MOVE : LED_PROCESSING);
	if (operation_id) {
		tracker_events_notify();
	}
}

static void sensitivity_failed(uint16_t operation_id, struct led_token feedback, uint8_t phase, uint8_t reason)
{
	k_mutex_lock(&sensitivity_lock, K_FOREVER);
	if (feedback.session != sensitivity_feedback.session || feedback.request_id != sensitivity_feedback.request_id) {
		k_mutex_unlock(&sensitivity_lock);
		return; /* Explicit set/reset already ended this measurement. */
	}
	cal_event_end(operation_id, CAL_OUTCOME_FAILED, phase, reason);
	sensor_calibration_result(feedback, LED_FAILED);
	if (sensitivity_operation == operation_id) {
		sensitivity_operation = 0;
		sensitivity_feedback = (struct led_token){0};
	}
	k_mutex_unlock(&sensitivity_lock);
	if (operation_id) {
		tracker_events_notify();
	}
}
#endif

int sensor_calibration_set_sensitivity(const float degrees[3])
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	if (!retained) {
		return sensor_operation_result(LED_OWNER_SENS, -ENODEV, false);
	}
	if (!degrees) {
		return sensor_operation_result(LED_OWNER_SENS, -EINVAL, false);
	}

	float scales[3];
	for (int i = 0; i < 3; i++) {
		if (!isfinite(degrees[i])) {
			return sensor_operation_result(LED_OWNER_SENS, -EINVAL, false);
		}
		float denominator = 1.0f - (degrees[i] / (360.0f * CONFIG_SENSOR_SENS_REV));
		if (!isfinite(denominator) || fabsf(denominator) < 1e-6f) {
			return sensor_operation_result(LED_OWNER_SENS, -EINVAL, false);
		}
		scales[i] = 1.0f / denominator;
		if (!isfinite(scales[i])) {
			return sensor_operation_result(LED_OWNER_SENS, -EINVAL, false);
		}
	}
	#if CONFIG_SENSOR_TCAL_HEATED
	int reserve_err = sensor_calibration_sensitivity_maintenance_begin();
	if (reserve_err) {
		return sensor_operation_result(LED_OWNER_SENS, reserve_err, false);
	}
	#endif
	sys_warm_transaction_begin();
	k_mutex_lock(&sensitivity_lock, K_FOREVER);
	bool notify = sensitivity_operation != 0;
	int err = sys_write(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, scales, sizeof(scales));
	sensitivity_replace_locked(CAL_REASON_REPLACED);
	k_mutex_unlock(&sensitivity_lock);
	sys_warm_transaction_end(false);
	#if CONFIG_SENSOR_TCAL_HEATED
	sensor_calibration_sensitivity_maintenance_end();
	#endif
	if (notify) {
		tracker_events_notify();
	}
	return sensor_operation_result(LED_OWNER_SENS, err, true);
#else
	ARG_UNUSED(degrees);
	return sensor_operation_result(LED_OWNER_SENS, -ENOTSUP, false);
#endif
}

int sensor_calibration_reset_sensitivity(void)
{
#if CONFIG_SENSOR_USE_SENS_CALIBRATION
	if (!retained) {
		return sensor_operation_result(LED_OWNER_SENS, -ENODEV, false);
	}
	float scales[3] = {1.0f, 1.0f, 1.0f};
	#if CONFIG_SENSOR_TCAL_HEATED
	int reserve_err = sensor_calibration_sensitivity_maintenance_begin();
	if (reserve_err) {
		return sensor_operation_result(LED_OWNER_SENS, reserve_err, false);
	}
	#endif
	sys_warm_transaction_begin();
	k_mutex_lock(&sensitivity_lock, K_FOREVER);
	bool notify = sensitivity_operation != 0;
	int err = sys_write(MAIN_GYRO_SENS_ID, &retained->gyroSensScale, scales, sizeof(scales));
	sensitivity_replace_locked(CAL_REASON_RESET);
	k_mutex_unlock(&sensitivity_lock);
	sys_warm_transaction_end(false);
	#if CONFIG_SENSOR_TCAL_HEATED
	sensor_calibration_sensitivity_maintenance_end();
	#endif
	if (notify) {
		tracker_events_notify();
	}
	return sensor_operation_result(LED_OWNER_SENS, err, true);
#else
	return sensor_operation_result(LED_OWNER_SENS, -ENOTSUP, false);
#endif
}

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
	const uint16_t operation_id = sensor_calibration_current_operation();
	const struct led_token feedback = sensor_calibration_current_feedback();
	const uint32_t collection_generation = sensor_calibration_current_generation();
	k_mutex_lock(&sensitivity_lock, K_FOREVER);
	const uint32_t generation = sensitivity_generation;
	sensitivity_operation = operation_id;
	sensitivity_feedback = feedback;
	k_mutex_unlock(&sensitivity_lock);
	uint8_t axis = sens_cal_axis;
	uint16_t revolutions = sens_cal_revolutions;

	sens_cal_report_axis = axis > 2 ? 0 : axis;
	sens_cal_report_deg = 0;

	if (axis > 2 || revolutions == 0) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_INVALID_PARAMS, 0);
		LOG_ERR("Sensitivity calibration: invalid parameters");
		printk("Gyro sensitivity auto-calibration failed: invalid parameters.\n");
		sensitivity_failed(operation_id, feedback, CAL_PHASE_WAIT_STILL, CAL_REASON_INVALID_ARGUMENT);
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
	sensor_calibration_stage(feedback, LED_WAIT_STILL);
	LOG_INF("Sensitivity calibration: hold still");
	sens_cal_publish(SENS_CAL_PHASE_HOLD_STILL, SENS_CAL_RESULT_NONE, 0);
	if (!wait_for_motion(false, 6)) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NOT_STILL, 0);
		LOG_WRN("Sensitivity calibration: tracker not still, aborting");
		printk("Gyro sensitivity auto-calibration failed: tracker was not still.\n");
		sensitivity_failed(operation_id, feedback, CAL_PHASE_WAIT_STILL, CAL_REASON_MOTION);
		return;
	}

	// 2. Measure the in-situ gyro bias. sensor_wait_gyro returns
	//    raw samples (before bias and sensitivity are applied), so we average a
	//    short window here rather than relying on the stored gyro bias.
	sens_cal_publish(SENS_CAL_PHASE_BIAS, SENS_CAL_RESULT_NONE, 0);
	sensitivity_step(operation_id, feedback, CAL_PHASE_COLLECT, 0);
	double bias_sum[3] = {0.0, 0.0, 0.0};
	int bias_count = 0;
	int64_t bias_start = k_uptime_get();
	while (k_uptime_get() - bias_start < SENS_CAL_BIAS_SAMPLE_MS) {
		if (sensor_wait_gyro(g, K_MSEC(1000))) {
			sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_GYRO_TIMEOUT, 0);
			LOG_WRN("Sensitivity calibration: gyro timeout during bias, aborting");
			printk("Gyro sensitivity auto-calibration failed: gyro timeout while measuring bias.\n");
			sensitivity_failed(operation_id, feedback, CAL_PHASE_COLLECT, CAL_REASON_SAMPLE_TIMEOUT);
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
		sensitivity_failed(operation_id, feedback, CAL_PHASE_COLLECT, CAL_REASON_INSUFFICIENT_SAMPLES);
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

	if (feedback.session) {
		k_mutex_lock(&sensitivity_lock, K_FOREVER);
		if (generation == sensitivity_generation && sensor_calibration_generation_valid(collection_generation)) {
			sensor_calibration_result(feedback, LED_STAGE_ACK);
		}
		k_mutex_unlock(&sensitivity_lock);
	}

	// 3. Arm and wait for the user to start spinning.
	LOG_INF("Sensitivity calibration: spin the tracker about the %c axis now", axis_char);
	sens_cal_publish(SENS_CAL_PHASE_ARMED, SENS_CAL_RESULT_NONE, 0);
	sensitivity_step(operation_id, feedback, CAL_PHASE_WAIT_ROTATION, axis);
	int64_t arm_start = k_uptime_get();
	int64_t last_wdt = arm_start;
	float rate = 0.0f;
	while (true) {
		if (k_uptime_get() - arm_start >= SENS_CAL_START_TIMEOUT_MS) {
			sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NO_SPIN, 0);
			LOG_WRN("Sensitivity calibration: no spin detected, aborting");
			printk("Gyro sensitivity auto-calibration failed: no spin detected.\n");
			sensitivity_failed(operation_id, feedback, CAL_PHASE_WAIT_ROTATION, CAL_REASON_START_TIMEOUT);
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

	// 4. Integrate the gyro rate over the spin.
	//    Use elapsed consumer time for integration, as before. The raw sample
	//    FIFO preserves vectors but does not carry acquisition timestamps.
	LOG_INF("Sensitivity calibration: recording");
	sens_cal_publish(SENS_CAL_PHASE_RECORDING, SENS_CAL_RESULT_NONE, 0);
	sensitivity_step(operation_id, feedback, CAL_PHASE_RECORD_ROTATION, axis);
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
			sensitivity_failed(operation_id, feedback, CAL_PHASE_RECORD_ROTATION, CAL_REASON_SAMPLE_TIMEOUT);
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
		sensitivity_failed(operation_id, feedback, CAL_PHASE_RECORD_ROTATION, CAL_REASON_RECORD_TIMEOUT);
		return;
	}

	sensitivity_step(operation_id, feedback, CAL_PHASE_VALIDATE, 0);
	float measured_deg = (float)fabs(measured);
	sens_cal_report_deg = measured_deg >= 65535.0f ? 65535 : (uint16_t)measured_deg;
	if (measured_deg < 1e-3f) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_ANGLE_TOO_SMALL, 0);
		LOG_WRN("Sensitivity calibration: measured angle too small, aborting");
		printk("Gyro sensitivity auto-calibration failed: measured angle too small.\n");
		sensitivity_failed(operation_id, feedback, CAL_PHASE_VALIDATE, CAL_REASON_QUALITY);
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
		sensitivity_failed(operation_id, feedback, CAL_PHASE_VALIDATE, CAL_REASON_INVALID_MODEL);
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
		sensitivity_failed(operation_id, feedback, CAL_PHASE_VALIDATE, CAL_REASON_QUALITY);
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
		sensitivity_failed(operation_id, feedback, CAL_PHASE_VALIDATE, CAL_REASON_INVALID_MODEL);
		return;
	}

	if (!retained) {
		sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_NO_RETAINED, sens_cal_scale_q12(scale));
		LOG_ERR("Sensitivity calibration: retained data unavailable, not applied");
		printk("Gyro sensitivity auto-calibration failed: retained data unavailable.\n");
		sensitivity_failed(operation_id, feedback, CAL_PHASE_APPLIED, CAL_REASON_SENSOR_UNAVAILABLE);
		return;
	}

	sys_warm_transaction_begin();
	k_mutex_lock(&sensitivity_lock, K_FOREVER);
	if (generation != sensitivity_generation || !sensor_calibration_generation_valid(collection_generation)) {
		bool cancelled = !sensor_calibration_generation_valid(collection_generation);
		k_mutex_unlock(&sensitivity_lock);
		sys_warm_transaction_end(false);
		if (cancelled) {
			cal_event_end(operation_id, CAL_OUTCOME_CANCELLED, CAL_PHASE_APPLY_PENDING, CAL_REASON_RESET);
			sensor_calibration_result(feedback, LED_CANCELLED);
			tracker_events_notify();
		}
		return;
	}
	retained->gyroSensScale[axis] = scale;
	cal_event_end(operation_id, CAL_OUTCOME_SUCCESS, CAL_PHASE_APPLIED, CAL_REASON_NONE);
	sensitivity_operation = 0;
	sensitivity_feedback = (struct led_token){0};
	retained_update();
	int storage_err = sys_write(MAIN_GYRO_SENS_ID, &retained->gyroSensScale,
				    retained->gyroSensScale, sizeof(retained->gyroSensScale));
	if (storage_err < 0) {
		cal_event_step(operation_id, CAL_PHASE_STORAGE, CAL_REASON_STORAGE_ERROR);
	}
	k_mutex_unlock(&sensitivity_lock);
	sys_warm_transaction_end(false);
	sensor_calibration_result(feedback, storage_err < 0 ? LED_APPLIED_NOT_SAVED : LED_SUCCESS);
	if (operation_id) {
		tracker_events_notify();
	}

	sens_cal_publish(SENS_CAL_PHASE_DONE, SENS_CAL_RESULT_OK, sens_cal_scale_q12(scale));
	LOG_INF("Sensitivity calibration: axis %c scale set to %.5f", axis_char, (double)scale);
	if (off_axis_ratio > SENS_CAL_WARN_OFF_AXIS_RATIO) {
		printk(
			"Gyro sensitivity auto-calibration applied: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, "
			"off-axis %.2f. Axis alignment was loose; repeating may improve accuracy.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	} else {
		printk(
			"Gyro sensitivity auto-calibration applied: axis %c, scale %.5f, equivalent sens diff over %u rev: %.3f deg, "
			"off-axis %.2f.\n",
			axis_char,
			(double)scale,
			(unsigned int)CONFIG_SENSOR_SENS_REV,
			(double)equivalent_diff_deg,
			(double)off_axis_ratio
		);
	}
	if (storage_err < 0) {
		LOG_ERR("Sensitivity calibration applied in RAM; persistence failed: %d", storage_err);
	}
}
#endif

#endif /* CONFIG_SENSOR_USE_SENS_CALIBRATION */
