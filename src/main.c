/* main.c - Magnetic shaft position/direction sensor
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shaft carries 4 magnets spaced 90 degrees apart. Two hall-effect sensors
 * (P0.08 = HALL_A, P0.11 = HALL_B) are mounted so that each passing magnet
 * triggers them in sequence rather than at the same instant. If HALL_A
 * fires and HALL_B follows within HALL_PAIR_TIMEOUT_MS, the shaft is
 * turning counter-clockwise; if HALL_B fires first, it is clockwise.
 * Each confirmed pair advances the position counter by one quarter turn.
 */

#include <zephyr/types.h>
#include <errno.h>
#include <stddef.h>
#include <device.h>
#include <devicetree.h>
#include <dt-bindings/gpio/gpio.h>
#include <drivers/adc.h>
#include <drivers/gpio.h>
#include <sys/atomic.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <kernel.h>
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
#include <hal/nrf_saadc.h>
#endif

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define GPIO0_NODE DT_NODELABEL(gpio0)
#define ADC_NODE DT_NODELABEL(adc)

#define HALL_A_PIN 8  /* leading sensor for counter-clockwise rotation */
#define HALL_B_PIN 11 /* leading sensor for clockwise rotation */

#define CAL_MIN_BUTTON_PIN 6 /* press at the 0% end of travel */
#define CAL_MAX_BUTTON_PIN 7 /* press at the 100% end of travel */

#define HALL_PAIR_TIMEOUT_MS 100 /* max time between paired edges; tune to top shaft speed */

/* CR2032 has no external divider; sample VDD directly via the internal SAADC input. */
#define BATTERY_SAMPLE_INTERVAL_MS (5 * 60 * MSEC_PER_SEC)

static const struct device *battery_adc = DEVICE_DT_GET(ADC_NODE);
static const struct adc_channel_cfg battery_adc_cfg = {
	.gain = ADC_GAIN_1_6,
	.reference = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
	.channel_id = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = NRF_SAADC_INPUT_VDD,
#endif
};

static const struct gpio_dt_spec hall_a = {
	.port = DEVICE_DT_GET(GPIO0_NODE),
	.pin = HALL_A_PIN,
	.dt_flags = (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH),
};
static const struct gpio_dt_spec hall_b = {
	.port = DEVICE_DT_GET(GPIO0_NODE),
	.pin = HALL_B_PIN,
	.dt_flags = (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH),
};
static const struct gpio_dt_spec cal_min_button = {
	.port = DEVICE_DT_GET(GPIO0_NODE),
	.pin = CAL_MIN_BUTTON_PIN,
	.dt_flags = (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH),
};
static const struct gpio_dt_spec cal_max_button = {
	.port = DEVICE_DT_GET(GPIO0_NODE),
	.pin = CAL_MAX_BUTTON_PIN,
	.dt_flags = (GPIO_PULL_DOWN | GPIO_ACTIVE_HIGH),
};

enum shaft_direction {
	DIRECTION_UNKNOWN = 0,
	DIRECTION_CW = 1,
	DIRECTION_CCW = 2,
};

enum hall_event {
	HALL_EVENT_NONE = 0,
	HALL_EVENT_A = 1,
	HALL_EVENT_B = 2,
};

struct __packed rot_adv_packet {
	uint16_t company_id;
	int32_t position_count;
	uint8_t position_percent;
	uint8_t direction;
	uint16_t battery_mv;
	uint8_t button_state;
};

static struct rot_adv_packet adv_packet = {
	.company_id = 0xFFFF,
};

/* Set Advertisement data */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, &adv_packet, sizeof(adv_packet)),
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static struct gpio_callback hall_irq_cb;
static struct gpio_callback cal_button_irq_cb;
static struct k_work adv_update_work;
static atomic_t position_count;
static atomic_t last_direction = ATOMIC_INIT(DIRECTION_UNKNOWN);
/* Calibration end points set by the min/max buttons; percent is 0 until max > min. */
static atomic_t cal_min_position;
static atomic_t cal_max_position;
static int16_t battery_sample_buffer;
static uint16_t battery_mv;
static int64_t last_battery_sample_ms = INT64_MIN;

/* Only accessed from the (serialized) GPIO ISR context, so no locking needed. */
static enum hall_event pending_event = HALL_EVENT_NONE;
static int64_t pending_event_ms;

static int sample_battery_mv(uint16_t *measured_mv)
{
	int ret;
	int32_t sample_mv;
	struct adc_sequence sequence = {
		.channels = BIT(0),
		.buffer = &battery_sample_buffer,
		.buffer_size = sizeof(battery_sample_buffer),
		.resolution = 12,
		.oversampling = 4,
	};

	ret = adc_read(battery_adc, &sequence);
	if (ret < 0) {
		return ret;
	}

	sample_mv = battery_sample_buffer;
	ret = adc_raw_to_millivolts(600, ADC_GAIN_1_6, 12, &sample_mv);
	if (ret < 0) {
		return ret;
	}

	*measured_mv = (uint16_t)CLAMP(sample_mv, 0, UINT16_MAX);

	return 0;
}

static void maybe_refresh_battery(bool force)
{
	int64_t now = k_uptime_get();
	uint16_t measured_mv;

	if (!force && ((now - last_battery_sample_ms) < BATTERY_SAMPLE_INTERVAL_MS)) {
		return;
	}

	if (sample_battery_mv(&measured_mv) == 0) {
		battery_mv = measured_mv;
		last_battery_sample_ms = now;
	}
}

static uint8_t compute_position_percent(int32_t position)
{
	int32_t min_pos = atomic_get(&cal_min_position);
	int32_t max_pos = atomic_get(&cal_max_position);
	int32_t range = max_pos - min_pos;
	int64_t pct;

	if (range <= 0) {
		return 0U;
	}

	pct = ((int64_t)(position - min_pos) * 100) / range;

	return (uint8_t)CLAMP(pct, 0, 100);
}

static uint8_t read_button_state(void)
{
	uint8_t state = 0U;

	state |= (gpio_pin_get_dt(&cal_min_button) > 0) ? BIT(0) : 0U;
	state |= (gpio_pin_get_dt(&cal_max_button) > 0) ? BIT(1) : 0U;

	return state;
}

static void update_adv_packet(void)
{
	int32_t position;

	maybe_refresh_battery(false);
	position = atomic_get(&position_count);
	adv_packet.position_count = position;
	adv_packet.position_percent = compute_position_percent(position);
	adv_packet.direction = (uint8_t)atomic_get(&last_direction);
	adv_packet.battery_mv = battery_mv;
	adv_packet.button_state = read_button_state();
#ifdef CONFIG_BT_DEBUG_LOG
	printk("pos=%d (%u%%) dir=%u batt=%umV btn=0x%x\n", adv_packet.position_count,
	       adv_packet.position_percent, adv_packet.direction, adv_packet.battery_mv,
	       adv_packet.button_state);
#endif
}

static void adv_update_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	update_adv_packet();
	(void)bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
}

/* Pairs a leading edge with the trailing edge from the other sensor to
 * determine direction. A/B order is: A then B -> CCW, B then A -> CW.
 */
static void handle_hall_edge(enum hall_event event, int64_t now_ms)
{
	if (event == HALL_EVENT_A) {
		if (pending_event == HALL_EVENT_B &&
		    (now_ms - pending_event_ms) <= HALL_PAIR_TIMEOUT_MS) {
			atomic_dec(&position_count);
			atomic_set(&last_direction, DIRECTION_CW);
			pending_event = HALL_EVENT_NONE;
		} else {
			pending_event = HALL_EVENT_A;
			pending_event_ms = now_ms;
		}
	} else if (event == HALL_EVENT_B) {
		if (pending_event == HALL_EVENT_A &&
		    (now_ms - pending_event_ms) <= HALL_PAIR_TIMEOUT_MS) {
			atomic_inc(&position_count);
			atomic_set(&last_direction, DIRECTION_CCW);
			pending_event = HALL_EVENT_NONE;
		} else {
			pending_event = HALL_EVENT_B;
			pending_event_ms = now_ms;
		}
	}
}

static void hall_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	int64_t now = k_uptime_get();

	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	if ((pins & BIT(hall_a.pin)) != 0U) {
		handle_hall_edge(HALL_EVENT_A, now);
	}
	if ((pins & BIT(hall_b.pin)) != 0U) {
		handle_hall_edge(HALL_EVENT_B, now);
	}

	k_work_submit(&adv_update_work);
}

static int configure_input(const struct gpio_dt_spec *spec)
{
	if (!device_is_ready(spec->port)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(spec, GPIO_INPUT);
}

/* Button A marks the current position as 0%; button B marks it as 100%. */
static void cal_button_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	int32_t position = atomic_get(&position_count);

	ARG_UNUSED(dev);
	ARG_UNUSED(cb);

	if ((pins & BIT(cal_min_button.pin)) != 0U) {
		atomic_set(&cal_min_position, position);
	}
	if ((pins & BIT(cal_max_button.pin)) != 0U) {
		atomic_set(&cal_max_position, position);
	}

	k_work_submit(&adv_update_work);
}

static void bt_ready(int err)
{
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
#ifdef CONFIG_BT_DEBUG_LOG
	printk("Position sensor started, advertising as %s\n", DEVICE_NAME);
#endif
}

void main(void)
{
	int err;

#ifdef CONFIG_BT_DEBUG_LOG
	printk("Starting Magnetic Position Sensor\n");
#endif

	if (!device_is_ready(battery_adc)) {
		printk("Battery ADC not ready\n");
		return;
	}

	err = adc_channel_setup(battery_adc, &battery_adc_cfg);
	if (err) {
		printk("Failed to configure battery ADC (err %d)\n", err);
		return;
	}

	err = configure_input(&hall_a);
	if (err) {
		printk("Failed to configure HALL_A (err %d)\n", err);
		return;
	}

	err = configure_input(&hall_b);
	if (err) {
		printk("Failed to configure HALL_B (err %d)\n", err);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&hall_a, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		printk("Failed to configure HALL_A interrupt (err %d)\n", err);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&hall_b, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		printk("Failed to configure HALL_B interrupt (err %d)\n", err);
		return;
	}

	gpio_init_callback(&hall_irq_cb, hall_irq_handler, BIT(hall_a.pin) | BIT(hall_b.pin));

	err = gpio_add_callback(hall_a.port, &hall_irq_cb);
	if (err) {
		printk("Failed to add HALL callback (err %d)\n", err);
		return;
	}

	err = configure_input(&cal_min_button);
	if (err) {
		printk("Failed to configure CAL_MIN button (err %d)\n", err);
		return;
	}

	err = configure_input(&cal_max_button);
	if (err) {
		printk("Failed to configure CAL_MAX button (err %d)\n", err);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&cal_min_button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		printk("Failed to configure CAL_MIN interrupt (err %d)\n", err);
		return;
	}

	err = gpio_pin_interrupt_configure_dt(&cal_max_button, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		printk("Failed to configure CAL_MAX interrupt (err %d)\n", err);
		return;
	}

	gpio_init_callback(&cal_button_irq_cb, cal_button_irq_handler,
			   BIT(cal_min_button.pin) | BIT(cal_max_button.pin));

	err = gpio_add_callback(cal_min_button.port, &cal_button_irq_cb);
	if (err) {
		printk("Failed to add calibration button callback (err %d)\n", err);
		return;
	}

	k_work_init(&adv_update_work, adv_update_work_handler);

	maybe_refresh_battery(true);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
}
