// SPDX-FileCopyrightText: 2026 airplay-esp32 contributors
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * USB Audio Class sink — the board enumerates as a USB speaker and plays
 * the host's audio through the same I2S DAC path AirPlay uses.
 */

/**
 * Called when the host's audio stream starts and stops.
 *
 * AirPlay and the USB sink cannot both drive I2S, so the owner of the
 * callback is expected to stop the AirPlay services on `true` and restart
 * them on `false`.  Invoked from the sink's writer task, so it may block.
 */
typedef void (*usb_audio_sink_state_cb_t)(bool streaming);

/**
 * Start the UAC device and the writer task that drains it into the DAC.
 */
esp_err_t usb_audio_sink_init(usb_audio_sink_state_cb_t state_cb);

/** DIAGNOSTIC (not for merging): the current stream's counters. */
typedef struct {
  bool streaming;
  uint32_t host_fps;    /**< frames per second the host delivered, last 2 s */
  uint32_t ring_filled; /**< bytes waiting in the ring buffer */
  uint32_t ring_size;
  uint32_t dropped;    /**< packets dropped because the ring was full */
  uint32_t underruns;  /**< times the ring ran dry and silence was sent */
  uint32_t missing_ms; /**< audio that never arrived from the host */
  uint32_t gap_max_us; /**< longest gap between two packets */
} usb_audio_sink_diag_t;
void usb_audio_sink_get_diag(usb_audio_sink_diag_t *out);

/**
 * True while the host is actively sending audio.
 */
bool usb_audio_sink_is_streaming(void);
