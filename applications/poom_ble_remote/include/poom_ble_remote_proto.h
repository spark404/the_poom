// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hugo Trippaers <hugo@trippaers.nl>

/**
 * @file poom_ble_remote_proto.h
 * @brief Wire format shared with the Pi 5 side (`poom-remote`, `src/proto.rs`).
 *
 * One custom GATT service with two characteristics:
 *
 * | characteristic | direction          | payload                            |
 * |----------------|--------------------|------------------------------------|
 * | buttons        | Poom -> Pi, notify | report below, on change and as a   |
 * |                |                    | heartbeat every 100 ms             |
 * | status         | Pi -> Poom, write  | reserved for robot state           |
 *
 * `buttons` payload, little endian:
 *
 * | offset | type | field                                       |
 * |--------|------|---------------------------------------------|
 * | 0      | u8   | protocol version, currently 1               |
 * | 1      | u8   | sequence number, +1 per notification, wraps |
 * | 2      | u16  | button bitmask                              |
 * | 4      | u8   | battery percent, 0xFF = unknown             |
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Canonical UUIDs, as written in the `poom-remote` README.
 *
 * - service: `7917cbf0-fa7b-4d8e-8aa3-f1f8f2cd4052`
 * - buttons: `7917cbf1-fa7b-4d8e-8aa3-f1f8f2cd4052`
 * - status : `7917cbf2-fa7b-4d8e-8aa3-f1f8f2cd4052`
 *
 * Bluedroid keeps `uuid128` least significant byte first, so the arrays below
 * are the canonical form reversed. The three UUIDs differ only in the byte that
 * reads `f0`/`f1`/`f2` above, which lands at index 12 once reversed.
 */
#define POOM_BLE_REMOTE_UUID128_LE(discriminator)                              \
    {                                                                          \
        0x52U, 0x40U, 0xcdU, 0xf2U, 0xf8U, 0xf1U, 0xa3U, 0x8aU,                \
        0x8eU, 0x4dU, 0x7bU, 0xfaU, (discriminator), 0xcbU, 0x17U, 0x79U       \
    }

/** @brief Discriminator byte of the service UUID. */
#define POOM_BLE_REMOTE_UUID_SERVICE_BYTE (0xf0U)
/** @brief Discriminator byte of the buttons characteristic UUID. */
#define POOM_BLE_REMOTE_UUID_BUTTONS_BYTE (0xf1U)
/** @brief Discriminator byte of the status characteristic UUID. */
#define POOM_BLE_REMOTE_UUID_STATUS_BYTE  (0xf2U)

/** @brief Advertised name, matched by the Pi when the service UUID is missing. */
#define POOM_BLE_REMOTE_DEVICE_NAME "poom-hexapod"

/** @brief Protocol version in byte 0 of every report. */
#define POOM_BLE_REMOTE_PROTOCOL_VERSION (1U)
/** @brief Length of one button report. The Pi accepts longer payloads. */
#define POOM_BLE_REMOTE_REPORT_LEN (5U)
/** @brief Battery byte when the Poom cannot measure its battery. */
#define POOM_BLE_REMOTE_BATTERY_UNKNOWN (0xFFU)

/**
 * @brief Bit positions in the report's button bitmask.
 *
 * These are the wire positions and differ from `button_layout_t` in the button
 * driver; `poom_ble_remote.c` maps between the two.
 */
typedef enum
{
    POOM_BLE_REMOTE_BIT_UP = 0,
    POOM_BLE_REMOTE_BIT_DOWN,
    POOM_BLE_REMOTE_BIT_LEFT,
    POOM_BLE_REMOTE_BIT_RIGHT,
    POOM_BLE_REMOTE_BIT_A,
    POOM_BLE_REMOTE_BIT_B,
    POOM_BLE_REMOTE_BIT_COUNT
} poom_ble_remote_bit_t;

/** @brief Mask of every bit the Pi knows about. */
#define POOM_BLE_REMOTE_BUTTON_MASK (0x003FU)

/**
 * @brief Encodes one button report into @p out.
 *
 * @param[out] out Buffer of at least `POOM_BLE_REMOTE_REPORT_LEN` bytes.
 * @param[in] seq Sequence number.
 * @param[in] buttons Button bitmask.
 * @param[in] battery Battery percent or `POOM_BLE_REMOTE_BATTERY_UNKNOWN`.
 * @return void
 */
static inline void poom_ble_remote_encode_report(uint8_t *out,
                                                 uint8_t seq,
                                                 uint16_t buttons,
                                                 uint8_t battery)
{
    buttons &= POOM_BLE_REMOTE_BUTTON_MASK;

    out[0] = POOM_BLE_REMOTE_PROTOCOL_VERSION;
    out[1] = seq;
    out[2] = (uint8_t)(buttons & 0x00FFU);
    out[3] = (uint8_t)((buttons >> 8) & 0x00FFU);
    out[4] = battery;
}

#ifdef __cplusplus
}
#endif
