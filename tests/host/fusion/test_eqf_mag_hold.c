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

/*
 * The magnetometer hold (ESB_PONG_FLAG_MAG_HOLD) has to actually stop the mag
 * pulling heading, not merely mark the field as disturbed. EqF is the easy place
 * to prove that: its state is file-static, it has no submodule dependency, and
 * its disturbance rejection only inflates the measurement sigma, so a hold that
 * set the flag without skipping the update would still let heading move.
 *
 * The check is: rotate the magnetic field far off its reference and feed it in.
 * Held, yaw must not move. Released, it must.
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "sensor/fusion/eqf/eqf.h"

#define GYRO_HZ 100.0f
#define DT (1.0f / GYRO_HZ)

/* Field pointing north and down, roughly Earth-like in gauss. */
#define MAG_NORTH 0.20f
#define MAG_DOWN 0.45f

static int failures;

static void check(bool ok, const char *what)
{
	printf("%s: %s\n", ok ? "PASS" : "FAIL", what);
	if (!ok) {
		failures++;
	}
}

static float quat_yaw_deg(const float q[4])
{
	float siny = 2.0f * (q[0] * q[3] + q[1] * q[2]);
	float cosy = 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]);
	return atan2f(siny, cosy) * 57.29577951308232f;
}

/* Field rotated by yaw_deg about the vertical axis, expressed in the body frame
 * of a tracker lying flat and level (body frame == earth frame here). */
static void mag_at_yaw(float yaw_deg, float m[3])
{
	float yaw = yaw_deg * 0.017453292519943295f;
	m[0] = MAG_NORTH * cosf(yaw);
	m[1] = -MAG_NORTH * sinf(yaw);
	m[2] = MAG_DOWN;
}

/* Feed a stationary, level tracker for the given duration. */
static void feed(float seconds, float field_yaw_deg)
{
	float g[3] = {0.0f, 0.0f, 0.0f};
	float a[3] = {0.0f, 0.0f, 1.0f};
	float m[3];

	mag_at_yaw(field_yaw_deg, m);
	int steps = (int)(seconds * GYRO_HZ);
	for (int i = 0; i < steps; i++) {
		eqf_update_gyro(g, DT);
		eqf_update_accel(a, DT);
		eqf_update_mag(m, DT);
	}
}

static float settle_and_get_yaw(void)
{
	float q[4];

	eqf_get_quat(q);
	return quat_yaw_deg(q);
}

int main(void)
{
	float held_drift, released_drift;

	eqf_init(DT, DT, DT);
	eqf_set_mag_hold(false);

	/* Converge on the undisturbed field. */
	feed(20.0f, 0.0f);
	float baseline = settle_and_get_yaw();

	/* Hold, then rotate the field 40 degrees. Nothing should follow it. */
	eqf_set_mag_hold(true);
	check(eqf_get_mag_hold(), "hold reads back as engaged");
	feed(20.0f, 40.0f);
	held_drift = fabsf(settle_and_get_yaw() - baseline);
	check(held_drift < 0.5f, "yaw does not follow the field while held");
	check(eqf_get_mag_dist_detected(), "held tracker reports mag disturbance");

	/* Release. The same field must now pull heading. */
	eqf_set_mag_hold(false);
	check(!eqf_get_mag_hold(), "hold reads back as released");
	feed(60.0f, 40.0f);
	released_drift = fabsf(settle_and_get_yaw() - baseline);
	check(released_drift > 5.0f, "yaw follows the field once released");

	printf("held drift %.3f deg, released drift %.3f deg\n", (double)held_drift, (double)released_drift);

	if (failures) {
		printf("%d check(s) failed\n", failures);
		return EXIT_FAILURE;
	}
	printf("all checks passed\n");
	return EXIT_SUCCESS;
}
