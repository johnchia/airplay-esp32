// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "pvdd_switch.h"

#include <stdatomic.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "pvdd_switch_logic.h"

#define ENABLE_GPIO ((gpio_num_t)CONFIG_PVDD_SWITCH_ENABLE_GPIO)
#define FAULT_GPIO  ((gpio_num_t)CONFIG_PVDD_SWITCH_FAULT_GPIO)
#define SENSE_GPIO  CONFIG_PVDD_SWITCH_SENSE_GPIO

#define POLL_MS       20U
#define BOOT_WAIT_MS  2000U
#define SENSE_SAMPLES 8
#define TASK_STACK    4096
#define TASK_PRIORITY 5

static const char TAG[] = "pvdd_switch";

static pvdd_switch_logic_t s_logic;
static pvdd_switch_supply_cb_t s_on_supply;
static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static adc_channel_t s_channel;
static bool s_ready;

// Set by the fault line's rising edge, so a fault shorter than the poll
// period still holds the switch off for PVDD_SWITCH_GOOD_MS.
static atomic_bool s_tripped;
static atomic_int s_sense_mv;
static atomic_bool s_enabled;
static atomic_bool s_fault;

static void IRAM_ATTR fault_isr(void *arg) {
  (void)arg;
  gpio_set_level(ENABLE_GPIO, 0);
  atomic_store(&s_tripped, true);
}

/* The supply in mV, from the divider on the sense pin. */
static int read_sense_mv(void) {
  int sum = 0;
  int count = 0;
  for (int i = 0; i < SENSE_SAMPLES; i++) {
    int raw = 0;
    if (adc_oneshot_read(s_adc, s_channel, &raw) != ESP_OK) {
      continue;
    }
    int pin_mv = 0;
    if (s_cali == NULL ||
        adc_cali_raw_to_voltage(s_cali, raw, &pin_mv) != ESP_OK) {
      // Uncalibrated: 12 dB attenuation reads to about 3.1 V at 12 bits.
      pin_mv = raw * 3100 / 4095;
    }
    sum += pin_mv;
    count++;
  }
  if (count == 0) {
    return 0;
  }
  const int64_t pin_mv = sum / count;
  const int64_t total = (int64_t)CONFIG_PVDD_SWITCH_SENSE_TOP_OHMS +
                        CONFIG_PVDD_SWITCH_SENSE_BOTTOM_OHMS;
  return (int)(pin_mv * total / CONFIG_PVDD_SWITCH_SENSE_BOTTOM_OHMS);
}

/* One period: read the pins, decide, drive the enable pin, report. Returns
 * the state after it. */
static pvdd_switch_state_t poll_once(uint32_t dt_ms) {
  const bool tripped = atomic_exchange(&s_tripped, false);
  const bool fault = tripped || gpio_get_level(FAULT_GPIO) != 0;
  const int sense_mv = read_sense_mv();
  const pvdd_switch_state_t before = s_logic.state;
  const pvdd_switch_out_t out =
      pvdd_switch_step(&s_logic, dt_ms, fault, sense_mv);

  // A fault that rose after the level was read has already pulled the pin
  // low, and the next period sees it; do not raise the pin over it.
  if (!out.enable || !atomic_load(&s_tripped)) {
    gpio_set_level(ENABLE_GPIO, out.enable ? 1 : 0);
    if (atomic_load(&s_tripped)) {
      gpio_set_level(ENABLE_GPIO, 0);
    }
  }
  atomic_store(&s_fault, fault);
  atomic_store(&s_enabled, out.enable);
  atomic_store(&s_sense_mv, sense_mv);

  if (before != PVDD_SWITCH_OFF && s_logic.state == PVDD_SWITCH_OFF) {
    ESP_LOGW(TAG, "Fault line high: amplifier supply switched off");
  } else if (before == PVDD_SWITCH_OFF && s_logic.state == PVDD_SWITCH_RAMP) {
    ESP_LOGI(TAG, "Contract good: amplifier supply switched on");
  }
  if (out.report) {
    if (out.supply_mv > 0) {
      ESP_LOGI(TAG, "Amplifier supply %.1f V", (float)out.supply_mv / 1000.0f);
    } else if (out.enable) {
      ESP_LOGW(TAG, "Switch on but the supply reads %.1f V",
               (float)sense_mv / 1000.0f);
    }
    if (s_on_supply != NULL) {
      s_on_supply(out.supply_mv);
    }
  }
  return s_logic.state;
}

static void pvdd_switch_task(void *arg) {
  (void)arg;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    (void)poll_once(POLL_MS);
  }
}

static esp_err_t sense_init(void) {
  adc_unit_t unit = ADC_UNIT_1;
  esp_err_t err = adc_oneshot_io_to_channel(SENSE_GPIO, &unit, &s_channel);
  if (err != ESP_OK) {
    return err;
  }
  const adc_oneshot_unit_init_cfg_t unit_cfg = {.unit_id = unit};
  err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
  if (err != ESP_OK) {
    return err;
  }
  const adc_oneshot_chan_cfg_t chan_cfg = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  err = adc_oneshot_config_channel(s_adc, s_channel, &chan_cfg);
  if (err != ESP_OK) {
    return err;
  }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
  const adc_cali_curve_fitting_config_t cali_cfg = {
      .unit_id = unit,
      .chan = s_channel,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) != ESP_OK) {
    s_cali = NULL;
  }
#endif
  if (s_cali == NULL) {
    ESP_LOGW(TAG, "No ADC calibration: the supply reading is approximate");
  }
  return ESP_OK;
}

esp_err_t pvdd_switch_init(pvdd_switch_supply_cb_t on_supply) {
  if (s_ready) {
    return ESP_ERR_INVALID_STATE;
  }
  s_on_supply = on_supply;
  pvdd_switch_logic_reset(&s_logic);

  // Low before it is an output: the board's pull-down has held the switch
  // off through reset, and it must not blip on here.
  gpio_set_level(ENABLE_GPIO, 0);
  const gpio_config_t enable_cfg = {
      .pin_bit_mask = 1ULL << ENABLE_GPIO,
      .mode = GPIO_MODE_OUTPUT,
  };
  esp_err_t err = gpio_config(&enable_cfg);
  if (err != ESP_OK) {
    return err;
  }
  gpio_set_level(ENABLE_GPIO, 0);

  // The chip drives the line. The pull-up makes a missing chip read as a
  // fault, so the switch stays off.
  const gpio_config_t fault_cfg = {
      .pin_bit_mask = 1ULL << FAULT_GPIO,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .intr_type = GPIO_INTR_POSEDGE,
  };
  err = gpio_config(&fault_cfg);
  if (err != ESP_OK) {
    return err;
  }
  err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    return err;
  }
  err = gpio_isr_handler_add(FAULT_GPIO, fault_isr, NULL);
  if (err != ESP_OK) {
    return err;
  }

  err = sense_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Supply sense on GPIO%d failed: %s", SENSE_GPIO,
             esp_err_to_name(err));
    return err;
  }

  // Give the amplifier its supply before it is initialised, if the charger
  // has agreed one. It usually has: the chip negotiates while the ESP32
  // boots.
  for (uint32_t waited = 0; waited < BOOT_WAIT_MS; waited += POLL_MS) {
    if (poll_once(POLL_MS) == PVDD_SWITCH_ON) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(POLL_MS));
  }
  if (s_logic.state != PVDD_SWITCH_ON) {
    ESP_LOGW(TAG,
             "No contract yet (fault line %s, supply %.1f V): the amplifier "
             "has no supply until the charger agrees one",
             gpio_get_level(FAULT_GPIO) ? "high" : "low",
             (float)atomic_load(&s_sense_mv) / 1000.0f);
  }

  if (xTaskCreate(pvdd_switch_task, "pvdd_switch", TASK_STACK, NULL,
                  TASK_PRIORITY, NULL) != pdPASS) {
    return ESP_ERR_NO_MEM;
  }
  s_ready = true;
  return ESP_OK;
}

esp_err_t pvdd_switch_get_status(pvdd_switch_status_t *status) {
  if (!s_ready || status == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  status->fault = atomic_load(&s_fault);
  status->enabled = atomic_load(&s_enabled);
  status->sense_mv = atomic_load(&s_sense_mv);
  status->supply_mv = s_logic.reported_mv;
  return ESP_OK;
}
