// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/**
 * @file pvdd_switch.h
 * @brief An amplifier supply (PVDD) behind a switch the firmware lets on
 *
 * For a board whose USB power-delivery chip negotiates by itself, from
 * resistor straps, and switches the amplifier's supply through a pair of
 * FETs that the ESP32 can hold off (the Loud-EH controller: CYPD3177). The
 * firmware does not talk to the chip here. It watches the chip's fault
 * line, lets the switch on only while that says the contract is good, and
 * measures the supply after the switch through a divider on an ADC pin.
 */

#include <stdbool.h>

#include "esp_err.h"

/** Told the amplifier's supply in mV whenever it changes; 0 = off. */
typedef void (*pvdd_switch_supply_cb_t)(int supply_mv);

/**
 * Take the pins, then wait up to about two seconds for a good contract and
 * the supply to come up, so the amplifier is initialised with its supply
 * present and known. A task goes on watching after that: the switch is
 * dropped the moment the fault line rises, and let on again when it has
 * been low for PVDD_SWITCH_GOOD_MS.
 *
 * @p on_supply hears the measured supply now, and after every later change.
 * Returns ESP_OK whether or not the supply came up in time.
 */
esp_err_t pvdd_switch_init(pvdd_switch_supply_cb_t on_supply);

/** What the switch is doing, for a status page or a log. */
typedef struct {
  bool fault;    /**< the fault line: no contract inside the window */
  bool enabled;  /**< the enable pin is high */
  int sense_mv;  /**< the supply as last measured */
  int supply_mv; /**< the supply as last reported, 0 = off or unknown */
} pvdd_switch_status_t;

esp_err_t pvdd_switch_get_status(pvdd_switch_status_t *status);
