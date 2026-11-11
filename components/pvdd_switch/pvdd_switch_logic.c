// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#include "pvdd_switch_logic.h"

#include <stdlib.h>

static int rounded(int sense_mv) {
  if (sense_mv < PVDD_SWITCH_MIN_MV) {
    return 0;
  }
  return (sense_mv + 50) / 100 * 100;
}

void pvdd_switch_logic_reset(pvdd_switch_logic_t *l) {
  l->state = PVDD_SWITCH_OFF;
  l->timer_ms = 0;
  l->change_ms = 0;
  l->reported_mv = 0;
}

pvdd_switch_out_t pvdd_switch_step(pvdd_switch_logic_t *l, uint32_t dt_ms,
                                   bool fault, int sense_mv) {
  pvdd_switch_out_t out = {0};

  if (fault) {
    // Off at once: the enable pin low is what keeps a charger's voltage
    // steps off the amplifier while there is no contract.
    out.report = l->reported_mv != 0;
    pvdd_switch_logic_reset(l);
    return out;
  }

  switch (l->state) {
  case PVDD_SWITCH_OFF:
    l->timer_ms += dt_ms;
    if (l->timer_ms >= PVDD_SWITCH_GOOD_MS) {
      l->state = PVDD_SWITCH_RAMP;
      l->timer_ms = 0;
    }
    break;

  case PVDD_SWITCH_RAMP:
    l->timer_ms += dt_ms;
    if (l->timer_ms >= PVDD_SWITCH_RAMP_MS) {
      l->state = PVDD_SWITCH_ON;
      l->change_ms = 0;
      l->reported_mv = rounded(sense_mv);
      out.report = true;
    }
    break;

  case PVDD_SWITCH_ON:
    if (abs(sense_mv - l->reported_mv) >= PVDD_SWITCH_CHANGE_MV) {
      l->change_ms += dt_ms;
      if (l->change_ms >= PVDD_SWITCH_CHANGE_MS) {
        l->change_ms = 0;
        l->reported_mv = rounded(sense_mv);
        out.report = true;
      }
    } else {
      l->change_ms = 0;
    }
    break;
  }

  out.enable = l->state != PVDD_SWITCH_OFF;
  out.supply_mv = l->reported_mv;
  return out;
}
