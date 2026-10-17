// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

// The real HUSB238A driver against a simulated chip and charger.

#include "mocks.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* The driver keeps its state in statics; compiling it in here lets every
 * case start from a fresh one. */
#include "husb238a.c"

#define ATTACH_DEBOUNCE 120 /* ms from ENABLE until the chip sees the cable */
#define OFFERS_DELAY    60  /* ms from GET_SRC_CAP until the offers land */
#define SETTLE_DELAY    250 /* ms from a request until the supply settles */
#define REJECT_BUSY     40
#define OWN_CONTRACT 90 /* ms from the offers until the chip's own contract */
#define LONG_AGO     (-100000)

static int bus_token, device_token, mutex_token;
static bool locked;

static struct {
  /* The board and the charger. */
  bool present;      /* the chip answers at 0x42 */
  int refuse_probes; /* probes it turns away before answering */
  bool attached;     /* something is on the cable */
  bool pd;           /* it speaks USB-PD */
  bool sends_offers; /* its offers arrive unasked once the chip is enabled */
  bool accepts;      /* it agrees to what it offers */
  int drop_gos;      /* commands the chip turns away as if busy */
  int offer_ma[HUSB238A_FIXED_PDOS];
  /* The chip. */
  uint8_t control1;
  uint8_t src_pdo;
  int contract; /* offer code in force: 0 plain USB, 1-5 fixed, 6+ others */
  int enabled_at;
  int offers_at;
  int pending;
  int settles_at;
  int busy_until;
  bool go_done; /* STATUS1.AMS_SUCC: the last command was carried out */
  /* The clock, which only vTaskDelay moves. */
  int now;
  /* What happened. */
  int probes;
  bool added;
  int get_caps;
  int selects[8];
  int n_selects;
  int ignored;
  int first_go_at;
  int changed_at; /* when a requested supply last settled */
  int supplies[8];
  int n_supplies;
} sim;

const char *esp_err_to_name(esp_err_t err) {
  (void)err;
  return "mock error";
}

void mock_log(const char *tag, const char *format, ...) {
  (void)tag;
  (void)format;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
  return &mutex_token;
}

void vSemaphoreDelete(SemaphoreHandle_t mutex) {
  assert(mutex == &mutex_token && !locked);
}

int xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout) {
  assert(mutex == &mutex_token && timeout == portMAX_DELAY && !locked);
  locked = true;
  return 1;
}

int xSemaphoreGive(SemaphoreHandle_t mutex) {
  assert(mutex == &mutex_token && locked);
  locked = false;
  return 1;
}

void vTaskDelay(uint32_t ticks) {
  sim.now += (int)ticks;
}

static bool chip_enabled(void) {
  return (sim.control1 & CONTROL1_ENABLE) != 0;
}

static bool sees_cable(void) {
  return chip_enabled() && sim.attached &&
         sim.now >= sim.enabled_at + ATTACH_DEBOUNCE;
}

static bool has_offers(void) {
  if (!sees_cable() || !sim.pd) {
    return false;
  }
  return sim.sends_offers || (sim.offers_at >= 0 && sim.now >= sim.offers_at);
}

static int code_volts(int code) {
  return code >= 1 && code <= HUSB238A_FIXED_PDOS
             ? husb238a_fixed_volts[code - 1]
             : 5;
}

/* Offers that arrive unasked make the chip settle on 5 V by itself, and
 * it is busy from the moment it sees the cable until that is done. */
static int own_contract_at(void) {
  return sim.enabled_at + ATTACH_DEBOUNCE + OWN_CONTRACT;
}

static bool makes_own_contract(void) {
  return sees_cable() && sim.pd && sim.sends_offers && sim.contract == 0 &&
         !sim.pending;
}

static bool busy(void) {
  return sim.now < sim.busy_until ||
         (makes_own_contract() && sim.now < own_contract_at());
}

static void settle(void) {
  if (sim.pending && sim.now >= sim.settles_at) {
    sim.contract = sim.pending;
    sim.pending = 0;
    sim.changed_at = sim.settles_at;
  }
  if (makes_own_contract() && sim.now >= own_contract_at()) {
    sim.contract = 1;
  }
}

static uint8_t reg_value(int reg) {
  if (reg >= 0x6A && reg <= 0x6E) {
    const int ma = sim.offer_ma[reg - 0x6A];
    return has_offers() && ma ? (uint8_t)(OFFER_DETECT | (ma / 100)) : 0;
  }
  switch (reg) {
  case REG_CONTROL1:
    return sim.control1;
  case REG_SRC_PDO:
    return sim.src_pdo;
  case REG_STATUS:
    return (uint8_t)((sees_cable() ? STATUS_ATTACH : 0) |
                     (busy() ? STATUS_AMS_BUSY : 0));
  case REG_STATUS + BLK_STATUS1:
    return (uint8_t)((has_offers() ? STATUS1_PD_COMM : 0) |
                     (sim.go_done ? STATUS1_AMS_SUCC : 0));
  case REG_STATUS + BLK_CONTRACT:
    return (uint8_t)(sim.contract << 4);
  default:
    return 0;
  }
}

esp_err_t i2c_master_probe(i2c_master_bus_handle_t bus, uint16_t address,
                           int timeout) {
  assert(bus == &bus_token && address == 0x42 && timeout > 0);
  sim.probes++;
  if (!sim.present || sim.refuse_probes > 0) {
    sim.refuse_probes--;
    return ESP_ERR_NOT_FOUND;
  }
  return ESP_OK;
}

esp_err_t board_i2c_add_device(i2c_master_bus_handle_t bus, uint8_t address,
                               uint32_t speed, i2c_master_dev_handle_t *dev) {
  assert(bus == &bus_token && address == 0x42 && speed <= 400000);
  assert(!sim.added);
  sim.added = true;
  *dev = &device_token;
  return ESP_OK;
}

esp_err_t board_i2c_remove_device(i2c_master_dev_handle_t dev) {
  assert(dev == &device_token && sim.added);
  sim.added = false;
  return ESP_OK;
}

esp_err_t board_i2c_read(i2c_master_dev_handle_t dev, uint8_t reg,
                         uint8_t *data, size_t size) {
  assert(dev == &device_token && sim.added);
  settle();
  for (size_t i = 0; i < size; i++) {
    data[i] = reg_value(reg + (int)i);
  }
  return ESP_OK;
}

esp_err_t board_i2c_write(i2c_master_dev_handle_t dev, uint8_t reg,
                          const uint8_t *data, size_t size) {
  assert(dev == &device_token && sim.added && size == 1);
  settle();
  const uint8_t value = data[0];
  if (reg == REG_CONTROL1) {
    if (!chip_enabled() && (value & CONTROL1_ENABLE)) {
      sim.enabled_at = sim.now;
    }
    sim.control1 = value;
  } else if (reg == REG_SRC_PDO) {
    /* Only ever one of the fixed 5-20 V offers: never PPS, AVS or EPR. */
    assert((value & 0x07) == 0);
    assert(value >> SRC_PDO_SHIFT >= 1 &&
           value >> SRC_PDO_SHIFT <= HUSB238A_FIXED_PDOS);
    sim.src_pdo = value;
  } else if (reg == REG_GO_COMMAND) {
    /* Never written on top of an exchange in flight. */
    assert(!busy());
    if (sim.first_go_at < 0) {
      sim.first_go_at = sim.now;
    }
    sim.go_done = chip_enabled() && sim.drop_gos == 0;
    if (sim.drop_gos > 0) {
      sim.drop_gos--;
      if (value == GO_SELECT_PDO) {
        sim.selects[sim.n_selects++] = code_volts(sim.src_pdo >> SRC_PDO_SHIFT);
      }
    } else if (!chip_enabled()) {
      sim.ignored++;
    } else if (value == GO_GET_SRC_CAP) {
      sim.get_caps++;
      if (sim.pd) {
        sim.offers_at = sim.now + OFFERS_DELAY;
      }
    } else if (value == GO_SELECT_PDO) {
      const int code = sim.src_pdo >> SRC_PDO_SHIFT;
      sim.selects[sim.n_selects++] = code_volts(code);
      if (sim.accepts && has_offers() && sim.offer_ma[code - 1]) {
        sim.pending = code;
        sim.settles_at = sim.now + SETTLE_DELAY;
        sim.busy_until = sim.settles_at;
      } else {
        sim.busy_until = sim.now + REJECT_BUSY;
      }
    } else {
      assert(!"unexpected command");
    }
  } else {
    assert(!"unexpected register write");
  }
  return ESP_OK;
}

static void on_supply(int supply_mv) {
  sim.supplies[sim.n_supplies++] = supply_mv;
}

/* A fresh driver and chip, and a charger that offers @p ma at 5-20 V. */
static void reset(int ma5, int ma9, int ma12, int ma15, int ma20) {
  memset(&sim, 0, sizeof(sim));
  sim.present = sim.attached = sim.pd = sim.sends_offers = sim.accepts = true;
  const int ma[HUSB238A_FIXED_PDOS] = {ma5, ma9, ma12, ma15, ma20};
  memcpy(sim.offer_ma, ma, sizeof(ma));
  sim.enabled_at = LONG_AGO;
  sim.offers_at = -1;
  sim.first_go_at = -1;
  s_dev = NULL;
  s_lock = NULL;
  s_on_supply = NULL;
  s_requested_volts = 0;
  s_reported_mv = 0;
  locked = false;
}

static void expect_ints(const char *what, const int *got, int n_got,
                        const int *want, int n_want) {
  bool same = n_got == n_want;
  for (int i = 0; same && i < n_got; i++) {
    same = got[i] == want[i];
  }
  if (!same) {
    fprintf(stderr, "%s:", what);
    for (int i = 0; i < n_got; i++) {
      fprintf(stderr, " %d", got[i]);
    }
    fprintf(stderr, " (wanted");
    for (int i = 0; i < n_want; i++) {
      fprintf(stderr, " %d", want[i]);
    }
    fprintf(stderr, ")\n");
  }
  assert(same);
}

#define EXPECT_SELECTS(...)                                  \
  do {                                                       \
    const int want[] = {__VA_ARGS__};                        \
    expect_ints("selects", sim.selects, sim.n_selects, want, \
                (int)(sizeof(want) / sizeof(want[0])));      \
  } while (0)
#define EXPECT_NO_SELECTS() assert(sim.n_selects == 0)
#define EXPECT_SUPPLIES(...)                                    \
  do {                                                          \
    const int want[] = {__VA_ARGS__};                           \
    expect_ints("supplies", sim.supplies, sim.n_supplies, want, \
                (int)(sizeof(want) / sizeof(want[0])));         \
  } while (0)

static void test_absent(void) {
  reset(3000, 3000, 3000, 3000, 3000);
  sim.present = false;
  assert(husb238a_init(&bus_token, 20, on_supply) == ESP_ERR_NOT_FOUND);
  assert(!husb238a_present() && !sim.added && sim.n_supplies == 0);
  assert(sim.probes == PROBE_TRIES);
  assert(husb238a_request(9) == ESP_ERR_INVALID_STATE);
  husb238a_status_t status;
  assert(husb238a_get_status(&status) == ESP_ERR_INVALID_STATE);
  assert(status.max_volts == CONFIG_HUSB238A_MAX_VOLTS && !status.pd);
  puts("ok   no chip: nothing added, nothing asked for, no supply reported");
}

static void test_first_probe_refused(void) {
  /* As on a 55 mm board: the first transaction after the bus comes up is
   * turned away, and every one after it answered. */
  reset(3000, 3000, 3000, 3000, 3000);
  sim.refuse_probes = 1;
  assert(husb238a_init(&bus_token, 9, on_supply) == ESP_OK);
  assert(husb238a_present() && sim.probes == 2);
  EXPECT_SELECTS(9);
  EXPECT_SUPPLIES(9000);

  reset(3000, 3000, 3000, 3000, 3000);
  sim.refuse_probes = PROBE_TRIES;
  assert(husb238a_init(&bus_token, 9, on_supply) == ESP_ERR_NOT_FOUND);
  assert(!sim.added && sim.probes == PROBE_TRIES);
  puts("ok   tries again when the chip turns away the first probe");
}

static void test_enables_then_asks(void) {
  reset(3000, 3000, 0, 3000, 3250);
  sim.control1 = 0x05; /* other bits, to be kept */
  assert(husb238a_init(&bus_token, 20, on_supply) == ESP_OK);
  assert(husb238a_present() && sim.control1 == (0x05 | CONTROL1_ENABLE));
  assert(sim.ignored == 0 && sim.get_caps == 0);
  /* Not before the chip's own contract, nor before ESPHome would ask; and
   * the new supply has settled by the time the amplifier is started. */
  assert(sim.first_go_at >= own_contract_at());
  assert(sim.first_go_at >= sim.enabled_at + FIRST_ASK_MS);
  assert(sim.now >= sim.changed_at + SUPPLY_SETTLE_MS);
#if CONFIG_HUSB238A_MAX_VOLTS >= 20
  EXPECT_SELECTS(20);
  EXPECT_SUPPLIES(20000);
#else
  EXPECT_SELECTS(15);
  EXPECT_SUPPLIES(15000);
#endif

  husb238a_status_t status;
  assert(husb238a_get_status(&status) == ESP_OK);
  assert(status.attached && status.pd);
  assert(status.offer_ma[0] == 3000 && status.offer_ma[2] == 0 &&
         status.offer_ma[4] == 3200);
  assert(status.supply_mv == sim.supplies[0]);
  assert(status.requested_volts == (CONFIG_HUSB238A_MAX_VOLTS >= 20 ? 20 : 15));
  puts("ok   enables the chip, keeps CONTROL1's other bits, asks for the top");
}

static void test_falls_back(void) {
  reset(3000, 3000, 0, 3000, 3000);
  assert(husb238a_init(&bus_token, 12, on_supply) == ESP_OK);
  EXPECT_SELECTS(9);
  EXPECT_SUPPLIES(9000);

  reset(3000, 3000, 3000, 3000, 3000);
  assert(husb238a_init(&bus_token, 10, on_supply) == ESP_OK);
  EXPECT_SELECTS(9);

  reset(3000, 3000, 3000, 3000, 3000);
  assert(husb238a_init(&bus_token, 3, on_supply) == ESP_OK);
  /* The chip's own contract is already the 5 V offer, at the charger's full
   * 5 V current. */
  EXPECT_NO_SELECTS();
  assert(sim.contract == 1);
  EXPECT_SUPPLIES(5000);
  puts("ok   falls back to the highest offer at or below the voltage asked");
}

static void test_asks_for_offers(void) {
  reset(3000, 3000, 3000, 3000, 3000);
  sim.sends_offers = false;
  assert(husb238a_init(&bus_token, 15, on_supply) == ESP_OK);
  assert(sim.get_caps == 1);
  EXPECT_SELECTS(15);
  EXPECT_SUPPLIES(15000);
  puts("ok   asks for the offers when they do not arrive by themselves");
}

static void test_not_pd(void) {
  reset(0, 0, 0, 0, 0);
  sim.pd = false;
  assert(husb238a_init(&bus_token, 20, on_supply) == ESP_OK);
  EXPECT_NO_SELECTS();
  EXPECT_SUPPLIES(5000);
  husb238a_status_t status;
  assert(husb238a_get_status(&status) == ESP_OK);
  assert(status.attached && !status.pd && status.supply_mv == 5000);

  reset(3000, 3000, 3000, 3000, 3000);
  sim.attached = false;
  assert(husb238a_init(&bus_token, 20, on_supply) == ESP_OK);
  EXPECT_NO_SELECTS();
  EXPECT_SUPPLIES(5000);
  puts("ok   plain USB and an empty cable stay at 5 V, asking for nothing");
}

static void test_warm_restart(void) {
  reset(3000, 3000, 3000, 3000, 3000);
  sim.control1 = CONTROL1_ENABLE;
  sim.contract = 4; /* 15 V, agreed before the restart */
  assert(husb238a_init(&bus_token, 15, on_supply) == ESP_OK);
  EXPECT_NO_SELECTS();
  EXPECT_SUPPLIES(15000);

  /* Something the board never asks for, such as PPS, is replaced. */
  reset(3000, 3000, 3000, 3000, 3000);
  sim.control1 = CONTROL1_ENABLE;
  sim.contract = 6;
  assert(husb238a_init(&bus_token, 9, on_supply) == ESP_OK);
  EXPECT_SELECTS(9);
  EXPECT_SUPPLIES(9000);
  puts("ok   keeps a contract in place, replaces one it never asks for");
}

static void test_dropped(void) {
  /* A command the chip turns away is sent again, once. */
  reset(3000, 3000, 3000, 3000, 3000);
  sim.drop_gos = 1;
  assert(husb238a_init(&bus_token, 9, on_supply) == ESP_OK);
  EXPECT_SELECTS(9, 9);
  EXPECT_SUPPLIES(9000);

  reset(3000, 3000, 3000, 3000, 3000);
  sim.drop_gos = SELECT_TRIES;
  assert(husb238a_init(&bus_token, 9, on_supply) == ESP_OK);
  assert(sim.n_selects == SELECT_TRIES);
  EXPECT_SUPPLIES(5000);

  /* A chip that stays busy is not written to at all. */
  reset(3000, 3000, 3000, 3000, 3000);
  sim.busy_until = 1000000;
  assert(husb238a_init(&bus_token, 9, on_supply) == ESP_ERR_TIMEOUT);
  EXPECT_NO_SELECTS();
  assert(sim.first_go_at < 0);
  puts("ok   resends a dropped request once, never writes to a busy chip");
}

static void test_changes(void) {
  reset(3000, 3000, 3000, 3000, 3000);
  assert(husb238a_init(&bus_token, 15, on_supply) == ESP_OK);
  assert(husb238a_request(9) == ESP_OK);  /* down */
  assert(husb238a_request(12) == ESP_OK); /* up */
  assert(husb238a_request(12) == ESP_OK); /* no change */
  EXPECT_SELECTS(15, 9, 12);
  /* Going down, the ceiling is lowered before the supply drops. */
  EXPECT_SUPPLIES(15000, 9000, 12000);
  husb238a_status_t status;
  assert(husb238a_get_status(&status) == ESP_OK);
  assert(status.requested_volts == 12 && status.supply_mv == 12000);

  assert(husb238a_request(10) == ESP_ERR_INVALID_ARG);
  assert(husb238a_request(4) == ESP_ERR_INVALID_ARG);
  assert(husb238a_request(28) == ESP_ERR_INVALID_ARG);
#if CONFIG_HUSB238A_MAX_VOLTS < 20
  assert(husb238a_request(20) == ESP_ERR_INVALID_ARG);
#endif
  assert(sim.n_selects == 3 && status.requested_volts == 12);
  puts("ok   changes voltage both ways, refuses voltages it cannot ask for");
}

static void test_refused(void) {
  reset(3000, 3000, 3000, 3000, 3000);
  sim.accepts = false;
  assert(husb238a_init(&bus_token, 15, on_supply) == ESP_OK);
  EXPECT_SELECTS(15);
  EXPECT_SUPPLIES(5000);

  /* Refused on the way down, the lowered ceiling goes back up. */
  reset(3000, 3000, 3000, 3000, 3000);
  sim.control1 = CONTROL1_ENABLE;
  sim.contract = 4;
  sim.accepts = false;
  assert(husb238a_init(&bus_token, 15, on_supply) == ESP_OK);
  assert(husb238a_request(9) == ESP_OK);
  EXPECT_SELECTS(9);
  EXPECT_SUPPLIES(15000, 9000, 15000);
  puts("ok   a refused request reports the supply that is really there");
}

int main(void) {
  printf("HUSB238A host tests (CONFIG_HUSB238A_MAX_VOLTS=%d)\n",
         CONFIG_HUSB238A_MAX_VOLTS);
  test_absent();
  test_first_probe_refused();
  test_enables_then_asks();
  test_falls_back();
  test_asks_for_offers();
  test_not_pd();
  test_warm_restart();
  test_dropped();
  test_changes();
  test_refused();
  return 0;
}
