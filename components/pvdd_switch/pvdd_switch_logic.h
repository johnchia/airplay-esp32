// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

/**
 * @file pvdd_switch_logic.h
 * @brief When the amplifier's supply switch is let on, and what supply is
 *        reported: the decisions only, with no pins or clock of their own.
 *
 * pvdd_switch.c reads the pins and calls pvdd_switch_step() on a fixed
 * period; a host test (tests/pvdd_switch) calls it with a made-up charger.
 */

#include <stdbool.h>
#include <stdint.h>

/** The fault line must stay low this long before the switch is let on. */
#define PVDD_SWITCH_GOOD_MS 200U
/** From letting the switch on to trusting the measured supply. */
#define PVDD_SWITCH_RAMP_MS 150U
/** A measured supply this far from the one reported, for
 * PVDD_SWITCH_CHANGE_MS, is reported again. USB-PD voltages are at least
 * 3 V apart, so a rail sagging under a bass note is not a new supply. */
#define PVDD_SWITCH_CHANGE_MV 1000
#define PVDD_SWITCH_CHANGE_MS 500U
/** Below this the supply is not a supply: reported as 0, unknown. */
#define PVDD_SWITCH_MIN_MV 4000

typedef enum {
  PVDD_SWITCH_OFF,  /**< held off: no good contract, or not for long enough */
  PVDD_SWITCH_RAMP, /**< let on, the supply still rising */
  PVDD_SWITCH_ON,   /**< let on, the supply reported */
} pvdd_switch_state_t;

typedef struct {
  pvdd_switch_state_t state;
  uint32_t timer_ms;  /**< time in OFF with no fault, or time in RAMP */
  uint32_t change_ms; /**< how long the measurement has differed, in ON */
  int reported_mv;    /**< the supply last reported, 0 = none or unknown */
} pvdd_switch_logic_t;

typedef struct {
  bool enable;   /**< drive the enable pin high */
  bool report;   /**< the supply changed: tell the amplifier supply_mv */
  int supply_mv; /**< rounded to 100 mV, 0 = off or unknown */
} pvdd_switch_out_t;

/** Start held off, with nothing reported. */
void pvdd_switch_logic_reset(pvdd_switch_logic_t *l);

/**
 * One period of @p dt_ms. @p fault is the power-delivery chip's fault line
 * (true: no contract inside the board's window), @p sense_mv the supply
 * measured after the switch.
 */
pvdd_switch_out_t pvdd_switch_step(pvdd_switch_logic_t *l, uint32_t dt_ms,
                                   bool fault, int sense_mv);
