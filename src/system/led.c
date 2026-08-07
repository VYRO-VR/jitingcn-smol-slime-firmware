#include "globals.h"

#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include "led.h"

LOG_MODULE_REGISTER(led, LOG_LEVEL_INF);

static void led_thread(void);
K_THREAD_DEFINE(led_thread_id, 512, led_thread, NULL, NULL, NULL, LED_THREAD_PRIORITY, 0, 0);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_en_gpios)
#define LED_EN_EXISTS true
static const struct gpio_dt_spec led_en = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_en_gpios);
#endif

#if CONFIG_LED_STRIP
#define LED_STRIP_EXISTS true
#include <zephyr/drivers/led_strip.h>
#define STRIP_NODE DT_ALIAS(led_strip)
static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, led_gpios)
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led0))
#ifndef LED_EXISTS
#define LED_EXISTS true
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#else
#define LED0_EXISTS true
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#endif
#endif
#ifndef LED_EXISTS
#ifndef LED_STRIP_EXISTS
#warning "LED GPIO does not exist"
// static const struct gpio_dt_spec led = {0};
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
#define LED1_EXISTS true
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
#define LED2_EXISTS true
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
#define LED3_EXISTS true
static const struct gpio_dt_spec led3 = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
#endif

#if DT_NODE_EXISTS(DT_ALIAS(pwm_led0))
#define PWM_LED_EXISTS true
static const struct pwm_dt_spec pwm_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
#else
#ifndef LED_STRIP_EXISTS
#warning "PWM LED node does not exist"
#endif
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led1))
#define PWM_LED1_EXISTS true
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
#endif
#if DT_NODE_EXISTS(DT_ALIAS(pwm_led2))
#define PWM_LED2_EXISTS true
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
#endif

static enum sys_led_pattern current_led_pattern;
static int current_priority;

#if LED_EXISTS || LED_STRIP_EXISTS
static enum sys_led_pattern led_patterns[SYS_LED_PATTERN_DEPTH]
	= {[0 ...(SYS_LED_PATTERN_DEPTH - 1)] = SYS_LED_PATTERN_OFF};
static int led_pattern_state;

static int led_pin_init(void)
{
	LOG_DBG("led_pin_init");
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 0);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_OUTPUT);
	gpio_pin_set_dt(&led0, 0);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_OUTPUT);
	gpio_pin_set_dt(&led1, 0);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_OUTPUT);
	gpio_pin_set_dt(&led2, 0);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_OUTPUT);
	gpio_pin_set_dt(&led3, 0);
#endif
	return 0;
}

SYS_INIT(led_pin_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void led_pin_reset(void)
{
	LOG_DBG("led_pin_reset");
#if LED_EXISTS
	gpio_pin_configure_dt(&led, GPIO_DISCONNECTED);
#endif
#if LED0_EXISTS
	gpio_pin_configure_dt(&led0, GPIO_DISCONNECTED);
#endif
#if LED1_EXISTS
	gpio_pin_configure_dt(&led1, GPIO_DISCONNECTED);
#endif
#if LED2_EXISTS
	gpio_pin_configure_dt(&led2, GPIO_DISCONNECTED);
#endif
#if LED3_EXISTS
	gpio_pin_configure_dt(&led3, GPIO_DISCONNECTED);
#endif
}

static void led_suspend(void)
{
	LOG_DBG("led_suspend");
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_SUSPEND);
#endif
	led_pin_reset();
	// disable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 0);
#endif
}

static void led_resume(void)
{
	LOG_DBG("led_resume");
	// enable power
#if LED_EN_EXISTS
	gpio_pin_configure_dt(&led_en, GPIO_OUTPUT);
	gpio_pin_set_dt(&led_en, 1);
#endif
#ifdef LED_STRIP_EXISTS
	pm_device_action_run(strip, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED_EXISTS
	pm_device_action_run(pwm_led.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED1_EXISTS
	pm_device_action_run(pwm_led1.dev, PM_DEVICE_ACTION_RESUME);
#endif
#ifdef PWM_LED2_EXISTS
	pm_device_action_run(pwm_led2.dev, PM_DEVICE_ACTION_RESUME);
#endif
	led_pin_init();
}

#ifdef LED_STRIP_EXISTS
#define LED_RGB_COLOR
#else
#ifdef CONFIG_LED_RGB_COLOR
#define LED_RGB_COLOR
#define LED_RG_COLOR
#endif

#if PWM_LED_EXISTS && PWM_LED1_EXISTS && PWM_LED2_EXISTS
#define LED_TRI_COLOR
#else
#undef LED_RGB_COLOR
#undef LED_TRI_COLOR
#if PWM_LED_EXISTS && PWM_LED1_EXISTS
#define LED_DUAL_COLOR
#else
#undef LED_RG_COLOR
#undef LED_DUAL_COLOR
#endif
#endif
#endif

#ifdef LED_RGB_COLOR
static int led_pwm_period[7][3] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G, CONFIG_LED_DEFAULT_COLOR_B}, // Default (purple)
	{0, 10000, 0},                                                                        // Success (green)
	{10000, 0, 0},                                                                        // Error (red)
	{8000, 3000, 0},                                                                      // Charging (amber)
	{0, 0, 10000},                                                                        // Pairing (blue)
	{10000, 8000, 0},                                                                     // Warning (yellow)
	{10000, 4500, 0},                                                                     // No-receiver (orange)
};
#elif defined(LED_TRI_COLOR)
static int led_pwm_period[7][3] = {
	{0, 0, 10000},    // Default
	{0, 10000, 0},    // Success
	{10000, 0, 0},    // Error
	{6000, 4000, 0},  // Charging
	{0, 0, 10000},    // Pairing
	{8000, 8000, 0},  // Warning (yellow)
	{10000, 4000, 0}, // No-receiver (orange)
};
#elif defined(LED_RG_COLOR)
static int led_pwm_period[7][2] = {
	{CONFIG_LED_DEFAULT_COLOR_R, CONFIG_LED_DEFAULT_COLOR_G}, // Default
	{0, 10000},                                               // Success
	{10000, 0},                                               // Error
	{8000, 2000},                                             // Charging
	{4000, 6000},                                             // Pairing
	{10000, 8000},                                            // Warning (yellow)
	{10000, 4000},                                            // No-receiver (orange)
};
#elif defined(LED_DUAL_COLOR)
static int led_pwm_period[7][2] = {
	{0, 10000},    // Default
	{0, 10000},    // Success
	{10000, 0},    // Error
	{6000, 4000},  // Charging
	{0, 10000},    // Pairing
	{10000, 4000}, // Warning
	{10000, 2000}, // No-receiver
};
#else
static int led_pwm_period[7][1] = {
	{10000}, // Default
	{10000}, // Success
	{10000}, // Error
	{10000}, // Charging
	{10000}, // Pairing
	{10000}, // Warning
	{10000}, // No-receiver
};
#endif

// Quadratic breath curve. Returns 0..peak following a smooth bell shape over
// 2*fade steps (fade-up then fade-down). Used by SYS_LED_PATTERN_* cases that
// step through fade phases with a fixed step time.
static inline int led_breath_value(int phase, int fade, int peak)
{
	if (phase < 0 || phase >= 2 * fade) {
		return 0;
	}
	int p = phase < fade ? phase : (2 * fade - phase);
	return (p * p * peak) / (fade * fade);
}

// Using brightness and value if PWM is supported, otherwise value is coerced to on/off
// TODO: use computed constants for high/low brightness and color values
static void led_pin_set(enum sys_led_color color, int brightness_pptt, int value_pptt)
{
	LOG_DBG("led_pin_set: color %d, brightness %d, value %d", color, brightness_pptt, value_pptt);
	if (brightness_pptt < 0) {
		brightness_pptt = 0;
	} else if (brightness_pptt > 10000) {
		brightness_pptt = 10000;
	}
	if (value_pptt < 0) {
		value_pptt = 0;
	} else if (value_pptt > 10000) {
		value_pptt = 10000;
	}
#if LED_STRIP_EXISTS
	static struct led_rgb pixel[1];
	value_pptt = value_pptt * brightness_pptt / 10000;
	pixel[0].r = 255 * (led_pwm_period[color][0] * value_pptt / 10000) / 10000;
	pixel[0].g = 255 * (led_pwm_period[color][1] * value_pptt / 10000) / 10000;
	pixel[0].b = 255 * (led_pwm_period[color][2] * value_pptt / 10000) / 10000;
	led_strip_update_rgb(strip, pixel, 1);
#elif PWM_LED_EXISTS
	value_pptt = value_pptt * brightness_pptt / 10000;
	// only supporting color if PWM is supported
	pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * (led_pwm_period[color][0] * value_pptt / 10000));
#if PWM_LED1_EXISTS
	pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * (led_pwm_period[color][1] * value_pptt / 10000));
#if PWM_LED2_EXISTS
	pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * (led_pwm_period[color][2] * value_pptt / 10000));
#endif
#endif
#else
	gpio_pin_set_dt(&led, value_pptt > 5000);
#endif
}
#endif

void set_led(enum sys_led_pattern led_pattern, int priority)
{
	LOG_DBG("set_led: current_led_pattern %d, current_priority %d", current_led_pattern, current_priority);
	LOG_DBG("set_led: pattern %d, priority %d", led_pattern, priority);
#if LED_EXISTS || LED_STRIP_EXISTS
	if (led_pattern <= SYS_LED_PATTERN_OFF && k_current_get() == led_thread_id) {
		led_patterns[current_priority] = led_pattern;
	} else {
		led_patterns[priority] = led_pattern;
	}
	for (priority = 0; priority < SYS_LED_PATTERN_DEPTH; priority++) {
		if (led_patterns[priority] == SYS_LED_PATTERN_OFF) {
			continue;
		}
		led_pattern = led_patterns[priority];
		break;
	}
	if (led_pattern == current_led_pattern && led_pattern > SYS_LED_PATTERN_OFF) {
		return;
	}
	current_led_pattern = led_pattern;
	current_priority = priority;
	led_pattern_state = 0;
	if (current_led_pattern <= SYS_LED_PATTERN_OFF) {
		led_suspend();
		k_thread_suspend(led_thread_id);
		LOG_DBG("set_led: suspended led_thread_id");
	} else if (k_current_get() != led_thread_id) // do not suspend if called from thread
	{
		k_thread_suspend(led_thread_id);
		LOG_DBG("set_led: suspended led_thread_id");
		led_resume();
		k_thread_resume(led_thread_id);
		k_wakeup(led_thread_id);
		LOG_DBG("set_led: resumed led_thread_id");
	} else {
		led_resume();
		k_thread_resume(led_thread_id);
		k_wakeup(led_thread_id);
		LOG_DBG("set_led: resumed led_thread_id");
	}
#endif
}

static void led_thread(void)
{
#if !LED_EXISTS && !LED_STRIP_EXISTS
	LOG_WRN("LED GPIO does not exist");
	return;
#else
	while (1) {
		LOG_DBG("led_thread: current_led_pattern %d", current_led_pattern);
		switch (current_led_pattern) {
		case SYS_LED_PATTERN_ON:
			led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 10000);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_SHORT:
			// Breathy pairing pulse: 200ms breath + 800ms rest @ 1Hz
			led_pattern_state++;
			if (led_pattern_state < 20) {
				led_pin_set(SYS_LED_COLOR_PAIRING, 10000,
							led_breath_value(led_pattern_state, 10, 10000));
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_PAIRING, 10000, 0);
				k_msleep(800);
				led_pattern_state = 0;
			}
			break;
		case SYS_LED_PATTERN_LONG:
			// Breathy waiting pulse: 600ms breath + 400ms rest @ 1Hz
			led_pattern_state++;
			if (led_pattern_state < 60) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000,
							led_breath_value(led_pattern_state, 30, 10000));
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
				k_msleep(400);
				led_pattern_state = 0;
			}
			break;
		case SYS_LED_PATTERN_FLASH:
			// Breathy quick pulse: 200ms breath + 200ms rest @ 2.5Hz
			led_pattern_state++;
			if (led_pattern_state < 20) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000,
							led_breath_value(led_pattern_state, 10, 10000));
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
				k_msleep(200);
				led_pattern_state = 0;
			}
			break;

		case SYS_LED_PATTERN_ONESHOT_POWERON:
			// Smooth 500ms fade-up, 200ms hold, 300ms fade-down.
			// Quadratic brightness ramp matches human perceptual response —
			// linear PWM scaling looks fast at the start and slow at the end.
			led_pattern_state++;
			if (led_pattern_state <= 100) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, led_pattern_state * led_pattern_state, 10000);
				k_msleep(5);
			} else if (led_pattern_state <= 140) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 10000);
				k_msleep(5);
			} else if (led_pattern_state <= 200) {
				int n = 200 - led_pattern_state; // 59 → 0
				led_pin_set(SYS_LED_COLOR_DEFAULT, n * n * 10000 / 3481, 10000);
				k_msleep(5);
			} else {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_POWEROFF:
			if (led_pattern_state++ > 0) {
				led_pin_set(
					SYS_LED_COLOR_DEFAULT,
					(202 - led_pattern_state) * 50,
					(led_pattern_state != 202 ? 10000 : 0)
				);
			} else {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
			}
			if (led_pattern_state == 202) {
				set_led(SYS_LED_PATTERN_OFF_FORCE, SYS_LED_PRIORITY_HIGHEST);
			} else if (led_pattern_state == 1) {
				k_msleep(250);
			} else {
				k_msleep(5);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_PROGRESS:
			// 2 breathy success pulses: each pulse is 200ms breath + 100ms gap.
			// 30-step slot per pulse, 60 steps total + finishing OFF call.
			led_pattern_state++;
			if (led_pattern_state > 60) {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
				break;
			}
			{
				int pos = (led_pattern_state - 1) % 30;
				int v = pos < 20 ? led_breath_value(pos, 10, 10000) : 0;
				led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, v);
				k_msleep(10);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_COMPLETE:
			// 4 breathy success pulses
			led_pattern_state++;
			if (led_pattern_state > 120) {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
				break;
			}
			{
				int pos = (led_pattern_state - 1) % 30;
				int v = pos < 20 ? led_breath_value(pos, 10, 10000) : 0;
				led_pin_set(SYS_LED_COLOR_SUCCESS, 10000, v);
				k_msleep(10);
			}
			break;
		case SYS_LED_PATTERN_ONESHOT_PING:
			// 10 breathy default-color pulses
			led_pattern_state++;
			if (led_pattern_state > 300) {
				set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_HIGHEST);
				break;
			}
			{
				int pos = (led_pattern_state - 1) % 30;
				int v = pos < 20 ? led_breath_value(pos, 10, 10000) : 0;
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, v);
				k_msleep(10);
			}
			break;

		case SYS_LED_PATTERN_ON_PERSIST:
			led_pin_set(SYS_LED_COLOR_SUCCESS, 2000, 10000);
			k_thread_suspend(led_thread_id);
			break;
		case SYS_LED_PATTERN_LONG_PERSIST:
			// Low battery: intermittent yellow flash. A crisp 150 ms blip
			// every 3 s — yellow (not red) signals "heads up, charge soon"
			// rather than "critical error", and the long rest saves a bit of
			// battery while the battery is already low.
			led_pattern_state++;
			if (led_pattern_state == 1) {
				led_pin_set(SYS_LED_COLOR_WARNING, 10000, 10000);
				k_msleep(150);
			} else {
				led_pin_set(SYS_LED_COLOR_WARNING, 10000, 0);
				k_msleep(2850);
				led_pattern_state = 0;
			}
			break;
		case SYS_LED_PATTERN_PULSE_PERSIST:
			led_pattern_state = (led_pattern_state + 1) % 1000;
			//			float led_value = sinf(led_pattern_state * (M_PI / 1000));
			//			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value * 10000);
			int led_value = led_pattern_state > 500 ? 1000 - led_pattern_state : led_pattern_state;
			if (led_value < 200) {
				led_value = (led_value) * 30;
			} else if (led_value < 300) {
				led_value = (led_value - 200) * 20 + 6000;
			} else if (led_value < 400) {
				led_value = (led_value - 300) * 15 + 8000;
			} else {
				led_value = (led_value - 400) * 5 + 9500;
			}
			led_pin_set(SYS_LED_COLOR_CHARGING, 10000, led_value);
			k_msleep(5);
			break;
		case SYS_LED_PATTERN_ACTIVE_PERSIST:
			// Breathy heartbeat: 300ms breath @ peak 40% + 9700ms rest
			led_pattern_state++;
			if (led_pattern_state < 30) {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000,
							led_breath_value(led_pattern_state, 15, 4000));
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_DEFAULT, 10000, 0);
				k_msleep(9700);
				led_pattern_state = 0;
			}
			break;

		case SYS_LED_PATTERN_ERROR_A:
			// 2 breathy red pulses then long pause = 5s cycle
			led_pattern_state++;
			if (led_pattern_state <= 60) {
				int pos = (led_pattern_state - 1) % 30;
				int v = pos < 20 ? led_breath_value(pos, 10, 10000) : 0;
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, v);
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, 0);
				k_msleep(4400);
				led_pattern_state = 0;
			}
			break;
		case SYS_LED_PATTERN_ERROR_B:
			// 3 breathy red pulses then pause = 5s cycle
			led_pattern_state++;
			if (led_pattern_state <= 90) {
				int pos = (led_pattern_state - 1) % 30;
				int v = pos < 20 ? led_breath_value(pos, 10, 10000) : 0;
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, v);
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, 0);
				k_msleep(4100);
				led_pattern_state = 0;
			}
			break;
		case SYS_LED_PATTERN_ERROR_C:
			// 4 breathy red pulses then pause = 5s cycle
			led_pattern_state++;
			if (led_pattern_state <= 120) {
				int pos = (led_pattern_state - 1) % 30;
				int v = pos < 20 ? led_breath_value(pos, 10, 10000) : 0;
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, v);
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, 0);
				k_msleep(3800);
				led_pattern_state = 0;
			}
			break;
		case SYS_LED_PATTERN_ERROR_D:
			// Continuous breathy red @ 1Hz
			led_pattern_state++;
			if (led_pattern_state < 60) {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000,
							led_breath_value(led_pattern_state, 30, 10000));
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, 0);
				k_msleep(400);
				led_pattern_state = 0;
			}
			break;

		case SYS_LED_PATTERN_HARDWARE_ERROR:
			// Two fast red flashes (100 ms on / 100 ms off) + 400 ms pause,
			// repeating. Urgent feel — reads as "the device itself is
			// broken (e.g. IMU not found), contact support". ~800 ms cycle.
			led_pattern_state++;
			if (led_pattern_state <= 4) {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000,
							led_pattern_state % 2 ? 10000 : 0);
				k_msleep(100);
			} else {
				led_pin_set(SYS_LED_COLOR_ERROR, 10000, 0);
				k_msleep(400);
				led_pattern_state = 0;
			}
			break;

		case SYS_LED_PATTERN_CRITICAL_ERROR:
			// Continuous very fast red flashing (100 ms on / 100 ms off).
			// Unmistakably "something is badly wrong".
			led_pattern_state ^= 1;
			led_pin_set(SYS_LED_COLOR_ERROR, 10000, led_pattern_state ? 10000 : 0);
			k_msleep(100);
			break;

		case SYS_LED_PATTERN_NO_RECEIVER:
			// Three crisp orange blinks (150 ms on / 150 ms off) + ~2 s gap
			// between each group. Clearly a connectivity issue ("searching
			// for receiver"), not a device fault.
			led_pattern_state++;
			if (led_pattern_state <= 6) {
				led_pin_set(SYS_LED_COLOR_NO_RECEIVER, 10000,
							led_pattern_state % 2 ? 10000 : 0);
				k_msleep(150);
			} else {
				led_pin_set(SYS_LED_COLOR_NO_RECEIVER, 10000, 0);
				k_msleep(2000);
				led_pattern_state = 0;
			}
			break;

		case SYS_LED_PATTERN_DFU:
			// DFU/OTA update mode: fast yellow pulse (100 ms breath +
			// 100 ms rest = 5 Hz). Fast enough to read as "busy updating,
			// do not power off".
			led_pattern_state++;
			if (led_pattern_state < 10) {
				led_pin_set(SYS_LED_COLOR_WARNING, 10000,
							led_breath_value(led_pattern_state, 5, 10000));
				k_msleep(10);
			} else {
				led_pin_set(SYS_LED_COLOR_WARNING, 10000, 0);
				k_msleep(100);
				led_pattern_state = 0;
			}
			break;

		case SYS_LED_PATTERN_RAINBOW_RAMP:
			// Pride rainbow cycle. 6 keyframes × 60 steps × 10 ms = 3.6 s
			// for one full hue rotation. Smooth cross-fade between
			// keyframes via linear interpolation. Bypasses the colour
			// table because we want full RGB control. 🏳️‍🌈
			led_pattern_state = (led_pattern_state + 1) % 360;
			{
				static const int keyframes[6][3] = {
					{10000,     0,     0}, // Red
					{10000,  5000,     0}, // Orange
					{10000, 10000,     0}, // Yellow
					{    0, 10000,     0}, // Green
					{    0,     0, 10000}, // Blue
					{ 8000,     0, 10000}, // Purple
				};
				int idx = led_pattern_state / 60;
				int t = led_pattern_state % 60;
				int next_idx = (idx + 1) % 6;
				int r = (keyframes[idx][0] * (60 - t) + keyframes[next_idx][0] * t) / 60;
				int g = (keyframes[idx][1] * (60 - t) + keyframes[next_idx][1] * t) / 60;
				int b = (keyframes[idx][2] * (60 - t) + keyframes[next_idx][2] * t) / 60;
				// 60 % brightness — comfortable to look at
				r = r * 60 / 100;
				g = g * 60 / 100;
				b = b * 60 / 100;
#if defined(PWM_LED_EXISTS)
				pwm_set_pulse_dt(&pwm_led, pwm_led.period / 10000 * r);
#endif
#if defined(PWM_LED1_EXISTS)
				pwm_set_pulse_dt(&pwm_led1, pwm_led1.period / 10000 * g);
#endif
#if defined(PWM_LED2_EXISTS)
				pwm_set_pulse_dt(&pwm_led2, pwm_led2.period / 10000 * b);
#endif
			}
			k_msleep(10);
			break;

		case SYS_LED_PATTERN_DRAIN_PERSIST:
			// Drive R+G+B PWM channels at peak (white) to maximise LED current.
			// Bypasses the colour table — this is intentionally outside the
			// normal palette because we want all three channels at 100 %.
			// Brief 200 ms off-pulse every 5 s for visual distinctiveness;
			// drain is reduced by ~4 % which is negligible.
			led_pattern_state = (led_pattern_state + 1) % 50; // 50 × 100 ms = 5 s cycle
			{
				bool dim = (led_pattern_state >= 48); // last 200 ms of cycle
#if defined(PWM_LED_EXISTS)
				pwm_set_pulse_dt(&pwm_led, dim ? 0 : pwm_led.period);
#endif
#if defined(PWM_LED1_EXISTS)
				pwm_set_pulse_dt(&pwm_led1, dim ? 0 : pwm_led1.period);
#endif
#if defined(PWM_LED2_EXISTS)
				pwm_set_pulse_dt(&pwm_led2, dim ? 0 : pwm_led2.period);
#endif
#if !defined(PWM_LED_EXISTS) && defined(LED_EXISTS)
				gpio_pin_set_dt(&led, dim ? 0 : 1);
#endif
			}
			k_msleep(100);
			break;

		default:
			LOG_DBG("led_thread: suspending led_thread_id");
			k_thread_suspend(led_thread_id);
		}
	}
#endif
}
