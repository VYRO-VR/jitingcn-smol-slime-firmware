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
 * The rest-gated heading check has to catch a field that rotates under a
 * stationary tracker without changing its norm — the case EqF's own detection
 * (norm only) cannot see. The check is: converge, then rotate the field about
 * the vertical while the tracker stays still. Yaw must not follow, and the
 * tracker must report a disturbance. Once the tracker moves and comes to rest
 * again the reference is re-latched, so the same field is then accepted.
 *
 * A rotation under the threshold must NOT trip it: that is ordinary noise and
 * reference drift, and yaw should still follow the field.
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

static float wrap_deg(float d)
{
	while (d > 180.0f) {
		d -= 360.0f;
	}
	while (d < -180.0f) {
		d += 360.0f;
	}
	return d;
}

/* Field rotated by yaw_deg about the vertical axis, expressed in the body frame
 * of a tracker lying flat and level (body frame == earth frame here). Norm and
 * dip are unchanged by construction, which is exactly what makes it invisible
 * to the norm check. */
static void mag_at_yaw(float yaw_deg, float m[3])
{
	float yaw = yaw_deg * 0.017453292519943295f;
	m[0] = MAG_NORTH * cosf(yaw);
	m[1] = -MAG_NORTH * sinf(yaw);
	m[2] = MAG_DOWN;
}

/* Feed a stationary, level tracker for the given duration. */
static void feed_still(float seconds, float field_yaw_deg)
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

/* Jiggle the tracker: alternating gyro readings with zero net rotation, enough
 * to break rest detection without turning the tracker. */
static void feed_jiggle(float seconds, float field_yaw_deg)
{
	float a[3] = {0.0f, 0.0f, 1.0f};
	float m[3];

	mag_at_yaw(field_yaw_deg, m);
	int steps = (int)(seconds * GYRO_HZ);
	for (int i = 0; i < steps; i++) {
		float g[3] = {0.0f, 0.0f, (i & 1) ? 20.0f : -20.0f};
		eqf_update_gyro(g, DT);
		eqf_update_accel(a, DT);
		eqf_update_mag(m, DT);
	}
}

static float yaw_now(void)
{
	float q[4];

	eqf_get_quat(q);
	return quat_yaw_deg(q);
}

int main(void)
{
	eqf_init(DT, DT, DT);

	/* Converge on the undisturbed field and settle into rest. */
	feed_still(20.0f, 0.0f);
	check(eqf_get_rest_detected(), "tracker is at rest after converging");
	check(!eqf_get_rest_heading_disturbed(), "no trip on a steady field");
	float baseline = yaw_now();

	/* Small rotation, under threshold: must not trip, and yaw should follow. */
	feed_still(30.0f, 2.0f);
	check(!eqf_get_rest_heading_disturbed(), "2 deg of field rotation does not trip the check");
	float small_drift = fabsf(wrap_deg(yaw_now() - baseline));
	check(small_drift > 1.0f, "yaw follows a sub-threshold field change");
	baseline = yaw_now();
	feed_still(5.0f, 2.0f);

	/* The real case: the field swings 10 deg while the tracker sits still. */
	feed_still(20.0f, 12.0f);
	check(eqf_get_rest_heading_disturbed(), "10 deg of field rotation at rest trips the check");
	check(eqf_get_mag_dist_detected(), "tripped check reports mag disturbance");
	float held_drift = fabsf(wrap_deg(yaw_now() - baseline));
	check(held_drift < 1.0f, "yaw does not follow the field while tripped");

	/* Leaving rest clears the trip; settling again latches the new field as
	 * the reference (accepted limitation), and heading follows it. */
	feed_jiggle(3.0f, 12.0f);
	check(!eqf_get_rest_detected(), "jiggle breaks rest detection");
	check(!eqf_get_rest_heading_disturbed(), "motion clears the trip");
	feed_still(60.0f, 12.0f);
	check(!eqf_get_rest_heading_disturbed(), "re-latched reference is not tripped");
	float released_drift = fabsf(wrap_deg(yaw_now() - baseline));
	check(released_drift > 5.0f, "yaw follows the field once re-latched");

	printf("small %.3f deg, tripped %.3f deg, re-latched %.3f deg\n",
	       (double)small_drift, (double)held_drift, (double)released_drift);

	if (failures) {
		printf("%d check(s) failed\n", failures);
		return EXIT_FAILURE;
	}
	printf("all checks passed\n");
	return EXIT_SUCCESS;
}
