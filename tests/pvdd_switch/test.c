// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

/* Host tests for components/pvdd_switch/pvdd_switch_logic.c: when the
 * amplifier's supply switch is let on and dropped, and which supply the
 * amplifier is told. */

#include "pvdd_switch_logic.h"

#include <stdio.h>

#define DT 20U

static int failures;

#define CHECK(cond, ...)                            \
  do {                                              \
    if (!(cond)) {                                  \
      failures++;                                   \
      printf("  FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);                          \
      printf("\n");                                 \
    }                                               \
  } while (0)

static pvdd_switch_logic_t l;
static int reports;
static int last_mv;
static bool enabled;

static void step(bool fault, int sense_mv) {
  const pvdd_switch_out_t out = pvdd_switch_step(&l, DT, fault, sense_mv);
  enabled = out.enable;
  if (out.report) {
    reports++;
    last_mv = out.supply_mv;
  }
}

static void run(uint32_t ms, bool fault, int sense_mv) {
  for (uint32_t t = 0; t < ms; t += DT) {
    step(fault, sense_mv);
  }
}

static void start(void) {
  pvdd_switch_logic_reset(&l);
  reports = 0;
  last_mv = -1;
  enabled = false;
}

// A good contract at mv, from reset to the supply reported.
static void bring_up(int mv) {
  start();
  run(PVDD_SWITCH_GOOD_MS, false, 0);
  run(PVDD_SWITCH_RAMP_MS + DT, false, mv);
}

static void test_no_contract(void) {
  printf("fault line high: never on, nothing reported\n");
  start();
  run(5000, true, 0);
  CHECK(!enabled && reports == 0, "enabled %d, reports %d", enabled, reports);
}

static void test_contract(void) {
  printf("good contract: on after %u ms, supply reported after the ramp\n",
         PVDD_SWITCH_GOOD_MS);
  start();
  run(PVDD_SWITCH_GOOD_MS - DT, false, 0);
  CHECK(!enabled, "on early");
  step(false, 0);
  CHECK(enabled && reports == 0, "enabled %d, reports %d", enabled, reports);
  run(PVDD_SWITCH_RAMP_MS - DT, false, 12000); // still rising
  CHECK(reports == 0, "reported during the ramp");
  run(2 * DT, false, 20040);
  CHECK(reports == 1 && last_mv == 20000, "%d reports, %d mV", reports,
        last_mv);
  run(5000, false, 19960);
  CHECK(enabled && reports == 1, "reported again with nothing changed");
}

static void test_fault_drops_at_once(void) {
  printf("fault while on: off the same period, supply reported gone\n");
  bring_up(20000);
  step(true, 20000);
  CHECK(!enabled && reports == 2 && last_mv == 0, "enabled %d, %d mV", enabled,
        last_mv);
  // A fault shorter than the wait still costs the whole wait.
  run(PVDD_SWITCH_GOOD_MS - DT, false, 0);
  CHECK(!enabled, "back on early");
  run(DT + PVDD_SWITCH_RAMP_MS + DT, false, 15000);
  CHECK(enabled && reports == 3 && last_mv == 15000, "%d reports, %d mV",
        reports, last_mv);
}

static void test_bouncing_fault(void) {
  printf("fault line bouncing: stays off\n");
  start();
  for (int i = 0; i < 50; i++) {
    run(PVDD_SWITCH_GOOD_MS - DT, false, 0);
    step(true, 0);
    CHECK(!enabled, "on during a bounce");
  }
  CHECK(reports == 0, "%d reports", reports);
}

static void test_sag_ignored(void) {
  printf("supply sagging under load: not a new supply\n");
  bring_up(20000);
  for (int i = 0; i < 20; i++) {
    run(PVDD_SWITCH_CHANGE_MS - DT, false, 18500); // a long bass note
    run(100, false, 20000);
  }
  run(5000, false, 19200); // under the step that counts
  CHECK(reports == 1 && last_mv == 20000, "%d reports, %d mV", reports,
        last_mv);
}

static void test_new_voltage(void) {
  printf("charger moves to another voltage with no fault: reported once\n");
  bring_up(20000);
  run(PVDD_SWITCH_CHANGE_MS, false, 9050);
  CHECK(reports == 2 && last_mv == 9100, "%d reports, %d mV", reports, last_mv);
  run(5000, false, 9050);
  CHECK(enabled && reports == 2, "%d reports", reports);
}

static void test_no_supply_behind_switch(void) {
  printf("contract good but nothing behind the switch: reported unknown\n");
  bring_up(300);
  CHECK(enabled && reports == 1 && last_mv == 0, "%d reports, %d mV", reports,
        last_mv);
  // It arrives later (the chip closed its own switch late).
  run(PVDD_SWITCH_CHANGE_MS, false, 20000);
  CHECK(reports == 2 && last_mv == 20000, "%d reports, %d mV", reports,
        last_mv);
}

int main(void) {
  test_no_contract();
  test_contract();
  test_fault_drops_at_once();
  test_bouncing_fault();
  test_sag_ignored();
  test_new_voltage();
  test_no_supply_behind_switch();
  if (failures) {
    printf("%d check(s) failed\n", failures);
    return 1;
  }
  printf("all passed\n");
  return 0;
}
