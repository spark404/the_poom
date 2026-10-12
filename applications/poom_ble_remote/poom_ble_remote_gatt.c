// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hugo Trippaers <hugo@trippaers.nl>

#include "poom_ble_remote_gatt.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "poom_ble_remote_proto.h"

// Logging
#define POOM_BLE_REMOTE_GATT_LOGGING
#ifdef POOM_BLE_REMOTE_GATT_LOGGING
    static const char *POOM_BLE_REMOTE_GATT_TAG = "poom_ble_remote_gatt";

    #define POOM_BLE_REMOTE_GATT_PRINTF_I(fmt, ...) \
        printf("[I] [%s] %s:%d: " fmt "\n", POOM_BLE_REMOTE_GATT_TAG, __func__, __LINE__, ##__VA_ARGS__)
    #define POOM_BLE_REMOTE_GATT_PRINTF_W(fmt, ...) \
        printf("[W] [%s] %s:%d: " fmt "\n", POOM_BLE_REMOTE_GATT_TAG, __func__, __LINE__, ##__VA_ARGS__)
    #define POOM_BLE_REMOTE_GATT_PRINTF_E(fmt, ...) \
        printf("[E] [%s] %s:%d: " fmt "\n", POOM_BLE_REMOTE_GATT_TAG, __func__, __LINE__, ##__VA_ARGS__)
#else
    #define POOM_BLE_REMOTE_GATT_PRINTF_I(...) do {} while (0)
    #define POOM_BLE_REMOTE_GATT_PRINTF_W(...) do {} while (0)
    #define POOM_BLE_REMOTE_GATT_PRINTF_E(...) do {} while (0)
#endif

// Defines
#define POOM_BLE_REMOTE_GATT_APP_ID (0x66U)
#define POOM_BLE_REMOTE_GATT_LOCAL_MTU (128)
// service declaration, two characteristic declarations, two characteristic
// values and one CCCD, rounded up
#define POOM_BLE_REMOTE_GATT_HANDLE_COUNT (8)
// Index of the byte that tells the three protocol UUIDs apart once Bluedroid
// has them in least-significant-byte-first order. See poom_ble_remote_proto.h.
#define POOM_BLE_REMOTE_GATT_UUID_DISCRIMINATOR (12U)

#define POOM_BLE_REMOTE_GATT_ADV_CONFIG_FLAG (1U << 0)
#define POOM_BLE_REMOTE_GATT_SCAN_RSP_CONFIG_FLAG (1U << 1)

// Type definitions
typedef struct
{
    esp_gatt_if_t gatts_if;
    uint16_t conn_id;
    uint16_t service_handle;
    uint16_t buttons_handle;
    uint16_t buttons_cccd_handle;
    uint16_t status_handle;
    bool connected;
    bool notify_enabled;
    uint8_t peer_bda[6];
} poom_ble_remote_gatt_ctx_t;

// Static variables
static const uint8_t s_poom_ble_remote_gatt_service_uuid[16] =
    POOM_BLE_REMOTE_UUID128_LE(POOM_BLE_REMOTE_UUID_SERVICE_BYTE);
static const uint8_t s_poom_ble_remote_gatt_buttons_uuid[16] =
    POOM_BLE_REMOTE_UUID128_LE(POOM_BLE_REMOTE_UUID_BUTTONS_BYTE);
static const uint8_t s_poom_ble_remote_gatt_status_uuid[16] =
    POOM_BLE_REMOTE_UUID128_LE(POOM_BLE_REMOTE_UUID_STATUS_BYTE);

// The 128-bit service UUID and the complete local name do not fit together in
// one 31-byte legacy advertisement, so the Pi matches on either: the UUID from
// the advertisement, the name from the scan response.
static uint8_t s_poom_ble_remote_gatt_adv_raw[3U + 2U + 16U] = {
    0x02U, ESP_BLE_AD_TYPE_FLAG, (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
    0x11U, ESP_BLE_AD_TYPE_128SRV_CMPL,
    // service UUID filled in by poom_ble_remote_gatt_build_adv_()
};
static uint8_t s_poom_ble_remote_gatt_scan_rsp_raw[2U + sizeof(POOM_BLE_REMOTE_DEVICE_NAME) - 1U];
static uint8_t s_poom_ble_remote_gatt_scan_rsp_len;

static esp_ble_adv_params_t s_poom_ble_remote_gatt_adv_params = {
    .adv_int_min = 0x20,   // 20 ms
    .adv_int_max = 0x40,   // 40 ms
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static poom_ble_remote_gatt_ctx_t s_poom_ble_remote_gatt_ctx;
static poom_ble_remote_gatt_cbs_t s_poom_ble_remote_gatt_cbs;
static uint8_t s_poom_ble_remote_gatt_adv_config_done;
static bool s_poom_ble_remote_gatt_started;

// Function prototypes
static void poom_ble_remote_gatt_build_adv_(void);
static void poom_ble_remote_gatt_reset_ctx_(void);
static void poom_ble_remote_gatt_add_char_(const uint8_t uuid128[16],
                                           esp_gatt_perm_t perm,
                                           esp_gatt_char_prop_t property);
static void poom_ble_remote_gatt_gap_cb_(esp_gap_ble_cb_event_t event,
                                         esp_ble_gap_cb_param_t *param);
static void poom_ble_remote_gatt_gatts_cb_(esp_gatts_cb_event_t event,
                                           esp_gatt_if_t gatts_if,
                                           esp_ble_gatts_cb_param_t *param);
static void poom_ble_remote_gatt_handle_write_(esp_gatt_if_t gatts_if,
                                               esp_ble_gatts_cb_param_t *param);

/**
 * @brief Fills the advertisement and scan response payloads.
 *
 * @return void
 */
static void poom_ble_remote_gatt_build_adv_(void)
{
    const char *name = POOM_BLE_REMOTE_DEVICE_NAME;
    const size_t name_len = sizeof(POOM_BLE_REMOTE_DEVICE_NAME) - 1U;

    (void)memcpy(&s_poom_ble_remote_gatt_adv_raw[5],
                 s_poom_ble_remote_gatt_service_uuid,
                 sizeof(s_poom_ble_remote_gatt_service_uuid));

    s_poom_ble_remote_gatt_scan_rsp_raw[0] = (uint8_t)(name_len + 1U);
    s_poom_ble_remote_gatt_scan_rsp_raw[1] = ESP_BLE_AD_TYPE_NAME_CMPL;
    (void)memcpy(&s_poom_ble_remote_gatt_scan_rsp_raw[2], name, name_len);
    s_poom_ble_remote_gatt_scan_rsp_len = (uint8_t)(name_len + 2U);
}

/**
 * @brief Returns all handles and link state to their unconnected defaults.
 *
 * @return void
 */
static void poom_ble_remote_gatt_reset_ctx_(void)
{
    (void)memset(&s_poom_ble_remote_gatt_ctx, 0x00, sizeof(s_poom_ble_remote_gatt_ctx));
    s_poom_ble_remote_gatt_ctx.gatts_if = ESP_GATT_IF_NONE;
}

/**
 * @brief Queues one characteristic for the service being built.
 *
 * Bluedroid adds characteristics one at a time, so the service is assembled
 * across ADD_CHAR_EVT / ADD_CHAR_DESCR_EVT rather than in one call.
 *
 * @param[in] uuid128 Characteristic UUID, least significant byte first.
 * @param[in] perm Attribute permissions.
 * @param[in] property Characteristic properties.
 * @return void
 */
static void poom_ble_remote_gatt_add_char_(const uint8_t uuid128[16],
                                           esp_gatt_perm_t perm,
                                           esp_gatt_char_prop_t property)
{
    esp_bt_uuid_t uuid = {0};
    esp_err_t ret;

    uuid.len = ESP_UUID_LEN_128;
    (void)memcpy(uuid.uuid.uuid128, uuid128, 16U);

    ret = esp_ble_gatts_add_char(s_poom_ble_remote_gatt_ctx.service_handle,
                                 &uuid,
                                 perm,
                                 property,
                                 NULL,
                                 NULL);
    if (ret != ESP_OK)
    {
        POOM_BLE_REMOTE_GATT_PRINTF_E("add char failed: 0x%x", ret);
    }
}

/**
 * @brief GAP callback.
 *
 * @param[in] event Event type.
 * @param[in] param Event parameters.
 * @return void
 */
static void poom_ble_remote_gatt_gap_cb_(esp_gap_ble_cb_event_t event,
                                         esp_ble_gap_cb_param_t *param)
{
    if (param == NULL)
    {
        return;
    }

    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            s_poom_ble_remote_gatt_adv_config_done &=
                (uint8_t)(~POOM_BLE_REMOTE_GATT_ADV_CONFIG_FLAG);
            if (s_poom_ble_remote_gatt_adv_config_done == 0U)
            {
                (void)esp_ble_gap_start_advertising(&s_poom_ble_remote_gatt_adv_params);
            }
            break;

        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            s_poom_ble_remote_gatt_adv_config_done &=
                (uint8_t)(~POOM_BLE_REMOTE_GATT_SCAN_RSP_CONFIG_FLAG);
            if (s_poom_ble_remote_gatt_adv_config_done == 0U)
            {
                (void)esp_ble_gap_start_advertising(&s_poom_ble_remote_gatt_adv_params);
            }
            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
            {
                POOM_BLE_REMOTE_GATT_PRINTF_E("advertising start failed status=0x%x",
                                              param->adv_start_cmpl.status);
            }
            else
            {
                POOM_BLE_REMOTE_GATT_PRINTF_I("advertising as %s", POOM_BLE_REMOTE_DEVICE_NAME);
            }
            break;

        default:
            break;
    }
}

/**
 * @brief Handles writes to the CCCD and to the status characteristic.
 *
 * @param[in] gatts_if GATT interface.
 * @param[in] param Event parameters.
 * @return void
 */
static void poom_ble_remote_gatt_handle_write_(esp_gatt_if_t gatts_if,
                                               esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;

    // Long writes would need a reassembly buffer; nothing in this protocol is
    // longer than an ATT MTU, so they are refused outright.
    if (param->write.is_prep)
    {
        status = ESP_GATT_REQ_NOT_SUPPORTED;
    }
    else if (param->write.handle == s_poom_ble_remote_gatt_ctx.buttons_cccd_handle)
    {
        if (param->write.len == 2U)
        {
            const uint16_t cccd = (uint16_t)(param->write.value[0] |
                                             ((uint16_t)param->write.value[1] << 8));

            s_poom_ble_remote_gatt_ctx.notify_enabled = ((cccd & 0x0001U) != 0U);
            POOM_BLE_REMOTE_GATT_PRINTF_I("notifications %s",
                                          s_poom_ble_remote_gatt_ctx.notify_enabled ? "on" : "off");
        }
        else
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    else if (param->write.handle == s_poom_ble_remote_gatt_ctx.status_handle)
    {
        // The Pi does not write this yet. Log it so the first payload that ever
        // arrives is visible instead of silently dropped.
        POOM_BLE_REMOTE_GATT_PRINTF_I("status write, %u bytes", (unsigned)param->write.len);
        for (uint16_t i = 0U; i < param->write.len; i++)
        {
            printf("%02x ", param->write.value[i]);
        }
        printf("\n");

        if (s_poom_ble_remote_gatt_cbs.on_status_write != NULL)
        {
            s_poom_ble_remote_gatt_cbs.on_status_write(param->write.value,
                                                       param->write.len,
                                                       s_poom_ble_remote_gatt_cbs.user_ctx);
        }
    }
    else
    {
        status = ESP_GATT_INVALID_HANDLE;
    }

    if (param->write.need_rsp)
    {
        (void)esp_ble_gatts_send_response(gatts_if,
                                          param->write.conn_id,
                                          param->write.trans_id,
                                          status,
                                          NULL);
    }
}

/**
 * @brief GATTS callback.
 *
 * @param[in] event Event type.
 * @param[in] gatts_if GATT interface.
 * @param[in] param Event parameters.
 * @return void
 */
static void poom_ble_remote_gatt_gatts_cb_(esp_gatts_cb_event_t event,
                                           esp_gatt_if_t gatts_if,
                                           esp_ble_gatts_cb_param_t *param)
{
    if (param == NULL)
    {
        return;
    }

    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            esp_gatt_srvc_id_t service_id = {0};
            esp_err_t ret;

            if (param->reg.status != ESP_GATT_OK)
            {
                POOM_BLE_REMOTE_GATT_PRINTF_E("app register failed status=%d", param->reg.status);
                break;
            }
            s_poom_ble_remote_gatt_ctx.gatts_if = gatts_if;

            service_id.is_primary = true;
            service_id.id.inst_id = 0U;
            service_id.id.uuid.len = ESP_UUID_LEN_128;
            (void)memcpy(service_id.id.uuid.uuid.uuid128,
                         s_poom_ble_remote_gatt_service_uuid,
                         sizeof(s_poom_ble_remote_gatt_service_uuid));

            ret = esp_ble_gatts_create_service(gatts_if,
                                               &service_id,
                                               POOM_BLE_REMOTE_GATT_HANDLE_COUNT);
            if (ret != ESP_OK)
            {
                POOM_BLE_REMOTE_GATT_PRINTF_E("create service failed: 0x%x", ret);
                break;
            }

            poom_ble_remote_gatt_build_adv_();
            s_poom_ble_remote_gatt_adv_config_done = POOM_BLE_REMOTE_GATT_ADV_CONFIG_FLAG |
                                                     POOM_BLE_REMOTE_GATT_SCAN_RSP_CONFIG_FLAG;
            (void)esp_ble_gap_config_adv_data_raw(s_poom_ble_remote_gatt_adv_raw,
                                                  sizeof(s_poom_ble_remote_gatt_adv_raw));
            (void)esp_ble_gap_config_scan_rsp_data_raw(s_poom_ble_remote_gatt_scan_rsp_raw,
                                                       s_poom_ble_remote_gatt_scan_rsp_len);
        }
        break;

        case ESP_GATTS_CREATE_EVT:
            s_poom_ble_remote_gatt_ctx.service_handle = param->create.service_handle;
            (void)esp_ble_gatts_start_service(s_poom_ble_remote_gatt_ctx.service_handle);

            poom_ble_remote_gatt_add_char_(s_poom_ble_remote_gatt_buttons_uuid,
                                           ESP_GATT_PERM_READ,
                                           ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY);
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
        {
            const uint8_t discriminator =
                param->add_char.char_uuid.uuid.uuid128[POOM_BLE_REMOTE_GATT_UUID_DISCRIMINATOR];

            if (param->add_char.status != ESP_GATT_OK)
            {
                POOM_BLE_REMOTE_GATT_PRINTF_E("add char status=%d", param->add_char.status);
                break;
            }

            if (discriminator == POOM_BLE_REMOTE_UUID_BUTTONS_BYTE)
            {
                esp_bt_uuid_t cccd_uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,
                };
                esp_err_t ret;

                s_poom_ble_remote_gatt_ctx.buttons_handle = param->add_char.attr_handle;

                ret = esp_ble_gatts_add_char_descr(s_poom_ble_remote_gatt_ctx.service_handle,
                                                   &cccd_uuid,
                                                   ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                   NULL,
                                                   NULL);
                if (ret != ESP_OK)
                {
                    POOM_BLE_REMOTE_GATT_PRINTF_E("add cccd failed: 0x%x", ret);
                }
            }
            else if (discriminator == POOM_BLE_REMOTE_UUID_STATUS_BYTE)
            {
                s_poom_ble_remote_gatt_ctx.status_handle = param->add_char.attr_handle;
                POOM_BLE_REMOTE_GATT_PRINTF_I("service ready buttons=%u cccd=%u status=%u",
                                              s_poom_ble_remote_gatt_ctx.buttons_handle,
                                              s_poom_ble_remote_gatt_ctx.buttons_cccd_handle,
                                              s_poom_ble_remote_gatt_ctx.status_handle);
            }
        }
        break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            s_poom_ble_remote_gatt_ctx.buttons_cccd_handle = param->add_char_descr.attr_handle;

            // The status characteristic goes in last so the two add_char calls
            // never overlap.
            poom_ble_remote_gatt_add_char_(s_poom_ble_remote_gatt_status_uuid,
                                           ESP_GATT_PERM_WRITE,
                                           ESP_GATT_CHAR_PROP_BIT_WRITE |
                                               ESP_GATT_CHAR_PROP_BIT_WRITE_NR);
            break;

        case ESP_GATTS_CONNECT_EVT:
        {
            esp_ble_conn_update_params_t conn_params = {0};

            s_poom_ble_remote_gatt_ctx.conn_id = param->connect.conn_id;
            s_poom_ble_remote_gatt_ctx.connected = true;
            (void)memcpy(s_poom_ble_remote_gatt_ctx.peer_bda,
                         param->connect.remote_bda,
                         sizeof(s_poom_ble_remote_gatt_ctx.peer_bda));

            // Ask for a connection interval well under the Pi's 300 ms
            // heartbeat timeout so a single lost packet cannot trip it.
            (void)memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.min_int = 0x0C;   // 15 ms
            conn_params.max_int = 0x18;   // 30 ms
            conn_params.latency = 0;
            conn_params.timeout = 400;    // 4 s
            (void)esp_ble_gap_update_conn_params(&conn_params);

            POOM_BLE_REMOTE_GATT_PRINTF_I("connected to %02x:%02x:%02x:%02x:%02x:%02x",
                                          param->connect.remote_bda[0], param->connect.remote_bda[1],
                                          param->connect.remote_bda[2], param->connect.remote_bda[3],
                                          param->connect.remote_bda[4], param->connect.remote_bda[5]);

            if (s_poom_ble_remote_gatt_cbs.on_connection != NULL)
            {
                s_poom_ble_remote_gatt_cbs.on_connection(true,
                                                         param->connect.remote_bda,
                                                         s_poom_ble_remote_gatt_cbs.user_ctx);
            }
        }
        break;

        case ESP_GATTS_DISCONNECT_EVT:
            POOM_BLE_REMOTE_GATT_PRINTF_I("disconnected reason=0x%x", param->disconnect.reason);
            s_poom_ble_remote_gatt_ctx.connected = false;
            s_poom_ble_remote_gatt_ctx.notify_enabled = false;
            (void)memset(s_poom_ble_remote_gatt_ctx.peer_bda,
                         0x00,
                         sizeof(s_poom_ble_remote_gatt_ctx.peer_bda));

            if (s_poom_ble_remote_gatt_cbs.on_connection != NULL)
            {
                s_poom_ble_remote_gatt_cbs.on_connection(false, NULL,
                                                         s_poom_ble_remote_gatt_cbs.user_ctx);
            }

            if (s_poom_ble_remote_gatt_started)
            {
                (void)esp_ble_gap_start_advertising(&s_poom_ble_remote_gatt_adv_params);
            }
            break;

        case ESP_GATTS_WRITE_EVT:
            poom_ble_remote_gatt_handle_write_(gatts_if, param);
            break;

        case ESP_GATTS_MTU_EVT:
            POOM_BLE_REMOTE_GATT_PRINTF_I("MTU=%d", param->mtu.mtu);
            break;

        default:
            break;
    }
}

esp_err_t poom_ble_remote_gatt_start(const poom_ble_remote_gatt_cbs_t *cbs)
{
    esp_err_t ret;

    if (s_poom_ble_remote_gatt_started)
    {
        return ESP_ERR_INVALID_STATE;
    }

    poom_ble_remote_gatt_reset_ctx_();
    s_poom_ble_remote_gatt_adv_config_done = 0U;
    if (cbs != NULL)
    {
        s_poom_ble_remote_gatt_cbs = *cbs;
    }
    else
    {
        (void)memset(&s_poom_ble_remote_gatt_cbs, 0x00, sizeof(s_poom_ble_remote_gatt_cbs));
    }

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
    {
        POOM_BLE_REMOTE_GATT_PRINTF_W("controller mem release classic failed: 0x%x", ret);
    }

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE)
    {
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        ret = esp_bt_controller_init(&cfg);
        if (ret != ESP_OK)
        {
            POOM_BLE_REMOTE_GATT_PRINTF_E("controller init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK)
        {
            POOM_BLE_REMOTE_GATT_PRINTF_E("controller enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED)
    {
        esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();

        ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg);
        if (ret != ESP_OK)
        {
            POOM_BLE_REMOTE_GATT_PRINTF_E("bluedroid init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    if (esp_bluedroid_get_status() != ESP_BLUEDROID_STATUS_ENABLED)
    {
        ret = esp_bluedroid_enable();
        if (ret != ESP_OK)
        {
            POOM_BLE_REMOTE_GATT_PRINTF_E("bluedroid enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    ret = esp_ble_gap_register_callback(poom_ble_remote_gatt_gap_cb_);
    if (ret != ESP_OK)
    {
        POOM_BLE_REMOTE_GATT_PRINTF_E("gap callback register failed: 0x%x", ret);
        return ret;
    }

    ret = esp_ble_gatts_register_callback(poom_ble_remote_gatt_gatts_cb_);
    if (ret != ESP_OK)
    {
        POOM_BLE_REMOTE_GATT_PRINTF_E("gatts callback register failed: 0x%x", ret);
        return ret;
    }

    // Advertising and service creation continue from ESP_GATTS_REG_EVT.
    ret = esp_ble_gatts_app_register(POOM_BLE_REMOTE_GATT_APP_ID);
    if (ret != ESP_OK)
    {
        POOM_BLE_REMOTE_GATT_PRINTF_E("gatts app register failed: 0x%x", ret);
        return ret;
    }

    ret = esp_ble_gatt_set_local_mtu(POOM_BLE_REMOTE_GATT_LOCAL_MTU);
    if (ret != ESP_OK)
    {
        POOM_BLE_REMOTE_GATT_PRINTF_W("set local mtu failed: 0x%x", ret);
    }

    s_poom_ble_remote_gatt_started = true;
    return ESP_OK;
}

void poom_ble_remote_gatt_stop(void)
{
    esp_err_t ret;

    if (!s_poom_ble_remote_gatt_started)
    {
        return;
    }

    // Cleared first so ESP_GATTS_DISCONNECT_EVT does not restart advertising
    // while the stack is coming down.
    s_poom_ble_remote_gatt_started = false;

    (void)esp_ble_gap_stop_advertising();

    if (s_poom_ble_remote_gatt_ctx.connected &&
        (s_poom_ble_remote_gatt_ctx.gatts_if != ESP_GATT_IF_NONE))
    {
        (void)esp_ble_gatts_close(s_poom_ble_remote_gatt_ctx.gatts_if,
                                  s_poom_ble_remote_gatt_ctx.conn_id);
    }

    if (s_poom_ble_remote_gatt_ctx.service_handle != 0U)
    {
        (void)esp_ble_gatts_stop_service(s_poom_ble_remote_gatt_ctx.service_handle);
        (void)esp_ble_gatts_delete_service(s_poom_ble_remote_gatt_ctx.service_handle);
    }

    if (s_poom_ble_remote_gatt_ctx.gatts_if != ESP_GATT_IF_NONE)
    {
        (void)esp_ble_gatts_app_unregister(s_poom_ble_remote_gatt_ctx.gatts_if);
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED)
    {
        ret = esp_bluedroid_disable();
        if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
        {
            POOM_BLE_REMOTE_GATT_PRINTF_W("bluedroid disable failed: 0x%x", ret);
        }
    }

    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED)
    {
        ret = esp_bluedroid_deinit();
        if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
        {
            POOM_BLE_REMOTE_GATT_PRINTF_W("bluedroid deinit failed: 0x%x", ret);
        }
    }

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
    {
        ret = esp_bt_controller_disable();
        if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
        {
            POOM_BLE_REMOTE_GATT_PRINTF_W("controller disable failed: 0x%x", ret);
        }
    }

    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED)
    {
        ret = esp_bt_controller_deinit();
        if ((ret != ESP_OK) && (ret != ESP_ERR_INVALID_STATE))
        {
            POOM_BLE_REMOTE_GATT_PRINTF_W("controller deinit failed: 0x%x", ret);
        }
    }

    poom_ble_remote_gatt_reset_ctx_();
    s_poom_ble_remote_gatt_adv_config_done = 0U;
    POOM_BLE_REMOTE_GATT_PRINTF_I("stopped");
}

bool poom_ble_remote_gatt_is_connected(void)
{
    return s_poom_ble_remote_gatt_ctx.connected;
}

bool poom_ble_remote_gatt_notifications_enabled(void)
{
    return s_poom_ble_remote_gatt_ctx.notify_enabled;
}

bool poom_ble_remote_gatt_peer_address(uint8_t out[6])
{
    if ((out == NULL) || !s_poom_ble_remote_gatt_ctx.connected)
    {
        return false;
    }

    (void)memcpy(out, s_poom_ble_remote_gatt_ctx.peer_bda, 6U);
    return true;
}

esp_err_t poom_ble_remote_gatt_notify_buttons(const uint8_t *report, uint16_t len)
{
    if ((report == NULL) || (len == 0U))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_poom_ble_remote_gatt_started ||
        !s_poom_ble_remote_gatt_ctx.connected ||
        !s_poom_ble_remote_gatt_ctx.notify_enabled ||
        (s_poom_ble_remote_gatt_ctx.buttons_handle == 0U))
    {
        return ESP_ERR_INVALID_STATE;
    }

    // need_confirm false: a notification, not an indication. An indication would
    // block on a round trip per report and cannot keep a 100 ms heartbeat.
    return esp_ble_gatts_send_indicate(s_poom_ble_remote_gatt_ctx.gatts_if,
                                       s_poom_ble_remote_gatt_ctx.conn_id,
                                       s_poom_ble_remote_gatt_ctx.buttons_handle,
                                       len,
                                       (uint8_t *)report,
                                       false);
}
