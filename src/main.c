
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

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <string.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>

#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
#include <nrfx_saadc.h>
#endif
#include <hal/nrf_ficr.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define GPIO0_NODE DT_NODELABEL(gpio0)
#define ADC_NODE DT_NODELABEL(adc)

#define HALL_A_PIN 8  /* leading sensor for counter-clockwise rotation */
#define HALL_B_PIN 11 /* leading sensor for clockwise rotation */

#define CAL_MIN_BUTTON_PIN 7 /* press at the 0% end of travel */
#define CAL_MAX_BUTTON_PIN 6 /* press at the 100% end of travel */

#define HALL_PAIR_TIMEOUT_MS 300 /* expire unmatched Hall edges */
#define HALL_REVERSE_MIN_PAIR_MS 15 /* suppress very short opposite-direction glitches */
#define ADV_BURST_DURATION_MS 250
#define ADV_HEARTBEAT_INTERVAL_MS (5 * MSEC_PER_SEC)
#define POSITION_SAVE_DELAY_MS (60 * MSEC_PER_SEC)
#define POSITION_STATE_NVS_ID 1U
#define POSITION_STATE_MAGIC 0x47524D50U

/* CR2032 has no external divider; sample VDD directly via the internal SAADC
 * input. */
#define BATTERY_SAMPLE_INTERVAL_MS (5 * 60 * MSEC_PER_SEC)

/* Measured offset correcting for internal reference/VDD-readback error;
 * bench comparison against a multimeter showed the raw conversion reads
 * ~75 mV low. */
#define BATTERY_CAL_OFFSET_MV 100

static const struct device *battery_adc = DEVICE_DT_GET(ADC_NODE);
static const struct adc_channel_cfg battery_adc_cfg = {
    .gain = ADC_GAIN_1_6,
    .reference = ADC_REF_INTERNAL,
    .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
    .channel_id = 0,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    /* NRFX_ANALOG_INTERNAL_VDD (nrfx_analog_common.h) replaces the old raw
     * NRF_SAADC_INPUT_VDD register constant - the nrfx SAADC driver now maps
     * ADC inputs through the generic nrfx_analog_input_t enum instead of
     * chip register values, and only recognizes the new enum. */
    .input_positive = NRFX_ANALOG_INTERNAL_VDD,
#endif
};

static const struct gpio_dt_spec hall_a = {
    .port = DEVICE_DT_GET(GPIO0_NODE),
    .pin = HALL_A_PIN,
    .dt_flags = (GPIO_ACTIVE_LOW),
};
static const struct gpio_dt_spec hall_b = {
    .port = DEVICE_DT_GET(GPIO0_NODE),
    .pin = HALL_B_PIN,
    .dt_flags = (GPIO_ACTIVE_LOW),
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

struct position_state {
  uint32_t magic;
  int32_t position_count;
  int32_t cal_min_position;
  int32_t cal_max_position;
};

static struct rot_adv_packet adv_packet = {
    .company_id = 0x5AFE,
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

static struct gpio_callback hall_a_irq_cb;
static struct gpio_callback hall_b_irq_cb;
static struct gpio_callback cal_button_irq_cb;
static struct k_work adv_update_work;
static struct k_work_delayable adv_stop_work;
static struct k_work_delayable adv_heartbeat_work;
static struct k_work_delayable position_save_work;
static struct nvs_fs position_nvs;
static atomic_t position_count;
static atomic_t last_direction = ATOMIC_INIT(DIRECTION_UNKNOWN);
static atomic_t bluetooth_ready;
/* Calibration end points set by the min/max buttons; percent is 0 until max >
 * min. */
static atomic_t cal_min_position;
static atomic_t cal_max_position;
static int16_t battery_sample_buffer;
static uint16_t battery_mv;
static int64_t last_battery_sample_ms = INT64_MIN;
static struct rot_adv_packet last_adv_packet;
static bool have_last_adv_packet;
static bool adv_active;

/* Only accessed from the serialized GPIO ISR context. */
static enum hall_event pending_event = HALL_EVENT_NONE;
static int64_t pending_event_ms;

static int init_position_storage(void) {
  struct flash_pages_info page_info;
  int err;

  position_nvs.flash_device = PARTITION_DEVICE(storage_partition);
  if (!device_is_ready(position_nvs.flash_device)) {
    return -ENODEV;
  }

  position_nvs.offset = PARTITION_OFFSET(storage_partition);
  err = flash_get_page_info_by_offs(position_nvs.flash_device,
                                    position_nvs.offset, &page_info);
  if (err) {
    return err;
  }

  position_nvs.sector_size = page_info.size;
  if ((PARTITION_SIZE(storage_partition) % page_info.size) != 0U) {
    return -EINVAL;
  }
  position_nvs.sector_count =
      PARTITION_SIZE(storage_partition) / page_info.size;
  if (position_nvs.sector_count < 2U) {
    return -EINVAL;
  }

  return nvs_mount(&position_nvs);
}

static int restore_position_state(void) {
  struct position_state state;
  ssize_t read_size;

  read_size = nvs_read(&position_nvs, POSITION_STATE_NVS_ID, &state,
                       sizeof(state));
  if (read_size == -ENOENT) {
    return 0;
  }
  if (read_size < 0) {
    return (int)read_size;
  }
  if ((size_t)read_size != sizeof(state) ||
      state.magic != POSITION_STATE_MAGIC) {
    return -EBADMSG;
  }

  atomic_set(&position_count, state.position_count);
  atomic_set(&cal_min_position, state.cal_min_position);
  atomic_set(&cal_max_position, state.cal_max_position);
  return 0;
}

static void position_save_work_handler(struct k_work *work) {
  struct position_state state = {
      .magic = POSITION_STATE_MAGIC,
      .position_count = atomic_get(&position_count),
      .cal_min_position = atomic_get(&cal_min_position),
      .cal_max_position = atomic_get(&cal_max_position),
  };
  ssize_t written;

  ARG_UNUSED(work);

  written = nvs_write(&position_nvs, POSITION_STATE_NVS_ID, &state,
                      sizeof(state));
  if (written < 0 ||
      (written != 0 && (size_t)written != sizeof(state))) {
    printk("Position state save failed (err %d)\n", (int)written);
  }
}

static void schedule_position_save(void) {
  (void)k_work_reschedule(&position_save_work,
                          K_MSEC(POSITION_SAVE_DELAY_MS));
}

static int sample_battery_mv(uint16_t *measured_mv) {
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

  sample_mv += BATTERY_CAL_OFFSET_MV;
  *measured_mv = (uint16_t)CLAMP(sample_mv, 0, UINT16_MAX);

  return 0;
}

static void maybe_refresh_battery(bool force) {
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

static uint8_t compute_position_percent(int32_t position) {
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

static uint8_t read_button_state(void) {
  uint8_t state = 0U;

  state |= (gpio_pin_get_dt(&cal_min_button) > 0) ? BIT(0) : 0U;
  state |= (gpio_pin_get_dt(&cal_max_button) > 0) ? BIT(1) : 0U;

  return state;
}

static void update_adv_packet(void) {
  int32_t position;

  maybe_refresh_battery(false);
  position = atomic_get(&position_count);
  adv_packet.position_count = position;
  adv_packet.position_percent = compute_position_percent(position);
  adv_packet.direction = (uint8_t)atomic_get(&last_direction);
  adv_packet.battery_mv = battery_mv;
  adv_packet.button_state = read_button_state();
}

static void advertise_packet_burst(bool force) {
  int err;

  if (!atomic_get(&bluetooth_ready)) {
    return;
  }

  update_adv_packet();
  if (!force && have_last_adv_packet &&
      memcmp(&adv_packet, &last_adv_packet, sizeof(adv_packet)) == 0) {
    return;
  }

  if (adv_active) {
    err = bt_le_adv_update_data(ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
  } else {
    /* BT_LE_ADV_OPT_USE_IDENTITY makes every burst restart advertise with
     * the fixed FICR-derived identity address instead of a fresh
     * non-resolvable private address, which is what bt_le_adv_start()
     * would otherwise generate on every single restart. */
    err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), sd,
                          ARRAY_SIZE(sd));
    if (err == 0) {
      adv_active = true;
    }
  }

  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Advertising update failed (err %d)\n", err);
#endif
    return;
  }

  memcpy(&last_adv_packet, &adv_packet, sizeof(adv_packet));
  have_last_adv_packet = true;
  (void)k_work_reschedule(&adv_stop_work, K_MSEC(ADV_BURST_DURATION_MS));
  (void)k_work_reschedule(&adv_heartbeat_work,
                          K_MSEC(ADV_HEARTBEAT_INTERVAL_MS));
#ifdef CONFIG_BT_DEBUG_LOG
  printk("ADV_BURST t=%lldms reason=%s ret=%d pos=%d pct=%u dir=%u "
         "batt=%umV btn=0x%x\n",
         (long long)k_uptime_get(), force ? "heartbeat" : "change", err,
         adv_packet.position_count,
         adv_packet.position_percent, adv_packet.direction,
         adv_packet.battery_mv, adv_packet.button_state);
#endif
}

static void adv_update_work_handler(struct k_work *work) {
  ARG_UNUSED(work);
  advertise_packet_burst(false);
}

static void adv_stop_work_handler(struct k_work *work) {
  int err;

  ARG_UNUSED(work);
  if (!adv_active) {
    return;
  }

  err = bt_le_adv_stop();
  if (err == 0) {
    adv_active = false;
  } else {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Advertising stop failed (err %d)\n", err);
#endif
  }
}

static void adv_heartbeat_work_handler(struct k_work *work) {
  ARG_UNUSED(work);
  advertise_packet_burst(true);
}

/* Pairs a leading edge with the trailing edge from the other sensor to
 * determine direction. A/B order is: A then B -> CCW, B then A -> CW.
 */
static void handle_hall_edge(enum hall_event event, int64_t now_ms) {
  int64_t pending_age_ms = now_ms - pending_event_ms;
  int64_t pair_delay_ms;
  bool reject_fast_reverse;

  if (pending_event != HALL_EVENT_NONE &&
      pending_age_ms <= HALL_PAIR_TIMEOUT_MS) {
    if (pending_event == event) {
      pending_event_ms = now_ms;
      return;
    }

    pair_delay_ms = pending_age_ms;
    if (pending_event == HALL_EVENT_B) {
      reject_fast_reverse =
          atomic_get(&last_direction) == DIRECTION_CCW &&
          pair_delay_ms < HALL_REVERSE_MIN_PAIR_MS;
      if (!reject_fast_reverse) {
        atomic_inc(&position_count);
        atomic_set(&last_direction, DIRECTION_CW);
        schedule_position_save();
      }
    } else {
      reject_fast_reverse =
          atomic_get(&last_direction) == DIRECTION_CW &&
          pair_delay_ms < HALL_REVERSE_MIN_PAIR_MS;
      if (!reject_fast_reverse) {
        atomic_dec(&position_count);
        atomic_set(&last_direction, DIRECTION_CCW);
        schedule_position_save();
      }
    }

    pending_event = HALL_EVENT_NONE;
  } else {
    pending_event = HALL_EVENT_NONE;
    pending_event = event;
    pending_event_ms = now_ms;
  }
}

static void hall_irq_process(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins) {
  int64_t now_ms = k_uptime_get();
  uint32_t hall_pins = pins & (BIT(hall_a.pin) | BIT(hall_b.pin));

  ARG_UNUSED(dev);
  ARG_UNUSED(cb);

  if (hall_pins == (BIT(hall_a.pin) | BIT(hall_b.pin))) {
    pending_event = HALL_EVENT_NONE;
    return;
  }

  if ((hall_pins & BIT(hall_a.pin)) != 0U) {
    handle_hall_edge(HALL_EVENT_A, now_ms);
  }
  if ((hall_pins & BIT(hall_b.pin)) != 0U) {
    handle_hall_edge(HALL_EVENT_B, now_ms);
  }

  k_work_submit(&adv_update_work);
}

static void hall_a_irq_handler(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins) {
  hall_irq_process(dev, cb, pins);
}

static void hall_b_irq_handler(const struct device *dev,
                               struct gpio_callback *cb, uint32_t pins) {
  hall_irq_process(dev, cb, pins);
}

static int configure_input(const struct gpio_dt_spec *spec) {
  if (!device_is_ready(spec->port)) {
    return -ENODEV;
  }

  return gpio_pin_configure_dt(spec, GPIO_INPUT);
}

/* Button A marks the current position as 0%; button B marks it as 100%. */
static void cal_button_irq_handler(const struct device *dev,
                                   struct gpio_callback *cb, uint32_t pins) {
  int32_t position = atomic_get(&position_count);
  bool state_changed = false;

  ARG_UNUSED(dev);
  ARG_UNUSED(cb);

  if ((pins & BIT(cal_min_button.pin)) != 0U) {
    /* Reset all calibration positions to 0 when the minimum button is pressed. This is an init state for calibration.
       The cal max position button should be triggered to set the open position count. */
    state_changed = atomic_get(&position_count) != 0 ||
                    atomic_get(&cal_min_position) != 0 ||
                    atomic_get(&cal_max_position) != 0;
    atomic_set(&position_count, 0);
    atomic_set(&cal_min_position, 0);
    atomic_set(&cal_max_position, 0);
  }
  if ((pins & BIT(cal_max_button.pin)) != 0U) {
    state_changed |= atomic_get(&cal_max_position) != position;
    atomic_set(&cal_max_position, position);
  }

  if (state_changed) {
    schedule_position_save();
  }
  k_work_submit(&adv_update_work);
}

/* Build a fixed BLE identity (random static) address from Nordic's
 * factory-programmed, chip-unique FICR DEVICEADDR registers, and register it
 * as the default identity before bt_enable(). Without this, bt_enable()
 * would generate a brand-new random static address from the CSPRNG on every
 * boot (this project has no settings/flash subsystem to persist one across
 * power cycles), so the advertised address would change every power-up.
 */
static int configure_static_ble_identity(void) {
  bt_addr_le_t addr = {.type = BT_ADDR_LE_RANDOM};
  uint32_t addr_lo = nrf_ficr_deviceaddr_get(NRF_FICR, 0);
  uint32_t addr_hi = nrf_ficr_deviceaddr_get(NRF_FICR, 1);

  addr.a.val[0] = (uint8_t)(addr_lo & 0xFF);
  addr.a.val[1] = (uint8_t)((addr_lo >> 8) & 0xFF);
  addr.a.val[2] = (uint8_t)((addr_lo >> 16) & 0xFF);
  addr.a.val[3] = (uint8_t)((addr_lo >> 24) & 0xFF);
  addr.a.val[4] = (uint8_t)(addr_hi & 0xFF);
  addr.a.val[5] = (uint8_t)((addr_hi >> 8) & 0xFF);
  /* Random static address: the two most significant bits must be 1. */
  addr.a.val[5] |= 0xC0;

  return bt_id_create(&addr, NULL);
}

static void bt_ready(int err) {
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Bluetooth init failed (err %d)\n", err);
#endif
    return;
  }

  printk("Bluetooth initialized\n");
  atomic_set(&bluetooth_ready, 1);
  k_work_submit(&adv_update_work);
#ifdef CONFIG_BT_DEBUG_LOG
  printk("Position sensor ready, advertising in bursts as %s\n", DEVICE_NAME);
#endif
}

int main(void) {
  int err;

#ifdef CONFIG_BT_DEBUG_LOG
  printk("Starting Magnetic Position Sensor\n");
#endif

  err = init_position_storage();
  if (err) {
    printk("Position storage initialization failed (err %d)\n", err);
    return err;
  }

  err = restore_position_state();
  if (err) {
    printk("Position state restore failed (err %d)\n", err);
    return err;
  }

  k_work_init(&adv_update_work, adv_update_work_handler);
  k_work_init_delayable(&adv_stop_work, adv_stop_work_handler);
  k_work_init_delayable(&adv_heartbeat_work, adv_heartbeat_work_handler);
  k_work_init_delayable(&position_save_work, position_save_work_handler);

  if (!device_is_ready(battery_adc)) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Battery ADC not ready\n");
#endif
    return -1;
  }

  err = adc_channel_setup(battery_adc, &battery_adc_cfg);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure battery ADC (err %d)\n", err);
#endif
    return -1;
  }

  err = configure_input(&hall_a);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure HALL_A (err %d)\n", err);
#endif
    return -1;
  }

  err = configure_input(&hall_b);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure HALL_B (err %d)\n", err);
#endif
    return -1;
  }

  err = gpio_pin_interrupt_configure_dt(&hall_a, GPIO_INT_EDGE_TO_ACTIVE);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure HALL_A interrupt (err %d)\n", err);
#endif
    return -1;
  }

  err = gpio_pin_interrupt_configure_dt(&hall_b, GPIO_INT_EDGE_TO_ACTIVE);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure HALL_B interrupt (err %d)\n", err);
#endif
    return -1;
  }

  gpio_init_callback(&hall_a_irq_cb, hall_a_irq_handler, BIT(hall_a.pin));

  err = gpio_add_callback(hall_a.port, &hall_a_irq_cb);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to add HALL_A callback (err %d)\n", err);
#endif
    return -1;
  }

  gpio_init_callback(&hall_b_irq_cb, hall_b_irq_handler, BIT(hall_b.pin));

  err = gpio_add_callback(hall_b.port, &hall_b_irq_cb);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to add HALL_B callback (err %d)\n", err);
#endif
    return -1;
  }

  err = configure_input(&cal_min_button);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure CAL_MIN button (err %d)\n", err);
#endif
    return -1;
  }

  err = configure_input(&cal_max_button);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure CAL_MAX button (err %d)\n", err);
#endif
    return -1;
  }

  err =
      gpio_pin_interrupt_configure_dt(&cal_min_button, GPIO_INT_EDGE_TO_ACTIVE);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure CAL_MIN interrupt (err %d)\n", err);
#endif
    return -1;
  }

  err =
      gpio_pin_interrupt_configure_dt(&cal_max_button, GPIO_INT_EDGE_TO_ACTIVE);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to configure CAL_MAX interrupt (err %d)\n", err);
#endif
    return -1;
  }

  gpio_init_callback(&cal_button_irq_cb, cal_button_irq_handler,
                     BIT(cal_min_button.pin) | BIT(cal_max_button.pin));

  err = gpio_add_callback(cal_min_button.port, &cal_button_irq_cb);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to add calibration button callback (err %d)\n", err);
#endif
    return -1;
  }

  maybe_refresh_battery(true);

  err = configure_static_ble_identity();
  if (err < 0) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Failed to set static BLE identity (err %d)\n", err);
    return -1;
#endif
  }

  /* Initialize the Bluetooth Subsystem */
  err = bt_enable(bt_ready);
  if (err) {
#ifdef CONFIG_BT_DEBUG_LOG
    printk("Bluetooth init failed (err %d)\n", err);
#endif
  }
}
