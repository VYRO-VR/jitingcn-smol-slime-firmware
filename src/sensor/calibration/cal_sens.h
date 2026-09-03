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
#ifndef SLIMENRF_CAL_SENS_H
#define SLIMENRF_CAL_SENS_H

#include <stdint.h>
#include <string.h>

/* Progress and outcome of a gyro sensitivity auto-calibration run.
 *
 * The calibration itself is a blocking procedure on the calibration thread that
 * reports only via printk to the tracker's own serial console. Publishing this
 * report lets the host follow the run over the radio instead of inferring the
 * outcome from phase timeouts.
 *
 * These enum values are part of the receiver/host wire contract (uplink
 * sub-packet type 6). Never renumber them, only append.
 */
enum sens_cal_phase {
	SENS_CAL_PHASE_IDLE = 0,
	SENS_CAL_PHASE_HOLD_STILL = 1, /* Waiting for the tracker to be still */
	SENS_CAL_PHASE_BIAS = 2,       /* Averaging the in-situ gyro bias */
	SENS_CAL_PHASE_ARMED = 3,      /* Armed, waiting for the spin to start */
	SENS_CAL_PHASE_RECORDING = 4,  /* Integrating the spin */
	SENS_CAL_PHASE_DONE = 5,       /* Terminal, see result */
};

enum sens_cal_result {
	SENS_CAL_RESULT_NONE = 0, /* Still running */
	SENS_CAL_RESULT_OK = 1,
	SENS_CAL_RESULT_INVALID_PARAMS = 2,
	SENS_CAL_RESULT_NOT_STILL = 3,
	SENS_CAL_RESULT_GYRO_TIMEOUT = 4,
	SENS_CAL_RESULT_NO_BIAS_SAMPLES = 5,
	SENS_CAL_RESULT_NO_SPIN = 6,
	SENS_CAL_RESULT_SPIN_TIMEOUT = 7,
	SENS_CAL_RESULT_ANGLE_TOO_SMALL = 8,
	SENS_CAL_RESULT_INVALID_SCALE = 9,
	SENS_CAL_RESULT_OFF_AXIS = 10,
	SENS_CAL_RESULT_SCALE_RANGE = 11,
	SENS_CAL_RESULT_NO_RETAINED = 12,
};

struct sens_cal_report {
	uint8_t phase;      /* enum sens_cal_phase */
	uint8_t result;     /* enum sens_cal_result */
	uint8_t axis;       /* 0 = X, 1 = Y, 2 = Z */
	uint8_t seq;        /* Incremented once per completed run */
	uint16_t scale_q12; /* Computed scale * 4096, 0 when not applicable */
	uint16_t progress;  /* Integrated |angle| in whole degrees, saturating */
};

#define SENS_CAL_SCALE_Q12_SHIFT 12

#if CONFIG_SENSOR_USE_SENS_CALIBRATION
/* Gyro sensitivity / scale calibration (from calibration.c). */
void sensor_calibrate_sens(void);

/* Snapshot of the current (or most recent) run. Safe to call from any thread. */
void sens_cal_get_report(struct sens_cal_report *out);
#else
/* The report shape stays available so the uplink path needs no conditional
 * compilation; without the feature it simply never leaves IDLE. */
static inline void sens_cal_get_report(struct sens_cal_report *out)
{
	memset(out, 0, sizeof(*out));
}
#endif

#endif /* SLIMENRF_CAL_SENS_H */
