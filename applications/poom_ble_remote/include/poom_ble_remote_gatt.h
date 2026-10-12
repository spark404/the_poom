// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hugo Trippaers <hugo@trippaers.nl>

/**
 * @file poom_ble_remote_gatt.h
 * @brief Bluedroid GATT peripheral for the hexapod remote protocol.
 *
 * The shared `poom_ble_gatt_server` module carries one characteristic and sends
 * indications; `poom_ble_gatt_dynamic` carries several but has no client
 * characteristic configuration descriptor at all. The remote needs two
 * characteristics and a real notification every 100 ms, so it owns its server,
 * the way `poom_ble_hid` owns its profile.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_gap_ble_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Called when the link to the Pi comes up or goes down.
 *
 * Runs on the Bluedroid callback task, not on the app task.
 *
 * @param[in] connected true on connect, false on disconnect.
 * @param[in] bda Peer address on connect, NULL on disconnect.
 * @param[in,out] user_ctx Opaque context passed to `poom_ble_remote_gatt_start`.
 * @return void
 */
typedef void (*poom_ble_remote_gatt_conn_cb_t)(bool connected,
                                               const uint8_t *bda,
                                               void *user_ctx);

/**
 * @brief Called for every write to the status characteristic.
 *
 * Runs on the Bluedroid callback task, not on the app task.
 *
 * @param[in] data Written payload.
 * @param[in] len Payload length.
 * @param[in,out] user_ctx Opaque context passed to `poom_ble_remote_gatt_start`.
 * @return void
 */
typedef void (*poom_ble_remote_gatt_status_cb_t)(const uint8_t *data,
                                                 uint16_t len,
                                                 void *user_ctx);

/**
 * @brief Optional callbacks. Any field may be NULL.
 */
typedef struct
{
    poom_ble_remote_gatt_conn_cb_t on_connection;
    poom_ble_remote_gatt_status_cb_t on_status_write;
    void *user_ctx;
} poom_ble_remote_gatt_cbs_t;

/**
 * @brief Brings up the BLE stack, publishes the service and starts advertising.
 *
 * @param[in] cbs Callback table, or NULL for none.
 * @return esp_err_t ESP_OK when the stack came up.
 */
esp_err_t poom_ble_remote_gatt_start(const poom_ble_remote_gatt_cbs_t *cbs);

/**
 * @brief Stops advertising, drops the link and tears the BLE stack back down.
 *
 * @return void
 */
void poom_ble_remote_gatt_stop(void);

/**
 * @brief Reports whether a peer is connected.
 *
 * @return bool
 */
bool poom_ble_remote_gatt_is_connected(void);

/**
 * @brief Reports whether the peer subscribed to the buttons characteristic.
 *
 * @return bool
 */
bool poom_ble_remote_gatt_notifications_enabled(void);

/**
 * @brief Copies the connected peer's address.
 *
 * @param[out] out Six-byte buffer.
 * @return bool false when no peer is connected; @p out is untouched.
 */
bool poom_ble_remote_gatt_peer_address(uint8_t out[6]);

/**
 * @brief Sends one button report as a notification.
 *
 * Does nothing unless a peer is connected and subscribed.
 *
 * @param[in] report Report bytes.
 * @param[in] len Report length.
 * @return esp_err_t ESP_OK when the notification was queued,
 *         ESP_ERR_INVALID_STATE when there is nobody to notify.
 */
esp_err_t poom_ble_remote_gatt_notify_buttons(const uint8_t *report, uint16_t len);

#ifdef __cplusplus
}
#endif
