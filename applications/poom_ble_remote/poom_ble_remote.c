// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hugo Trippaers <hugo@trippaers.nl>

#include "poom_ble_remote.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Arduboy2.h"
#include "bsp_pong.h"
#include "button_driver.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input_events.h"
#include "poom_ble_remote_gatt.h"
#include "poom_ble_remote_proto.h"
#include "poom_led_rainbow.h"
#include "poom_sbus.h"
#include "ws2812.h"

// Logging
#define POOM_BLE_REMOTE_LOGGING
#ifdef POOM_BLE_REMOTE_LOGGING
    static const char *POOM_BLE_REMOTE_TAG = "poom_ble_remote";

    #define POOM_BLE_REMOTE_PRINTF_D(fmt, ...) \
        printf("[D] [%s] %s:%d: " fmt "\n", POOM_BLE_REMOTE_TAG, __func__, __LINE__, ##__VA_ARGS__)
#else
    #define POOM_BLE_REMOTE_PRINTF_D(fmt, ...) do {} while (0)
#endif

// Defines
#ifndef POOM_MENU_RESUME_TOPIC
#define POOM_MENU_RESUME_TOPIC "poom/menu/resume"
#endif

#define POOM_BLE_REMOTE_TASK_STACK (4096U)
#define POOM_BLE_REMOTE_TASK_PRIO (5U)

// The task wakes often so a button change reaches the Pi quickly; the screen
// and the LEDs refresh far more slowly so they never hold up a report.
#define POOM_BLE_REMOTE_TICK_MS (10U)
#define POOM_BLE_REMOTE_HEARTBEAT_MS (100U)
#define POOM_BLE_REMOTE_DRAW_MS (150U)

// A and B held together for this long leaves the app. B on its own stays a
// protocol button (the Pi maps it to toggle_stand), so it cannot also be exit.
#define POOM_BLE_REMOTE_EXIT_HOLD_MS (1000U)

// Sentinel in place of the last sent bitmask. Six bits are ever set, so this
// never compares equal and forces a report as soon as the Pi subscribes.
#define POOM_BLE_REMOTE_NO_REPORT_SENT (0xFFFFU)

#define POOM_BLE_REMOTE_LED_BRIGHTNESS (26U)   // 10% of full scale
#define POOM_BLE_REMOTE_LED_RESOLUTION_HZ (10 * 1000 * 1000)
#define POOM_BLE_REMOTE_LED_BLINK_MS (500U)

// Type definitions
typedef void (*poom_ble_remote_exit_cb_t)(void *user_ctx);

// Wire bit per button_layout_t index. The driver numbers buttons A, B, LEFT,
// RIGHT, UP, DOWN; the protocol numbers them up, down, left, right, A, B.
static const uint8_t s_poom_ble_remote_wire_bit[] = {
    [BUTTON_A] = (uint8_t)POOM_BLE_REMOTE_BIT_A,
    [BUTTON_B] = (uint8_t)POOM_BLE_REMOTE_BIT_B,
    [BUTTON_LEFT] = (uint8_t)POOM_BLE_REMOTE_BIT_LEFT,
    [BUTTON_RIGHT] = (uint8_t)POOM_BLE_REMOTE_BIT_RIGHT,
    [BUTTON_UP] = (uint8_t)POOM_BLE_REMOTE_BIT_UP,
    [BUTTON_DOWN] = (uint8_t)POOM_BLE_REMOTE_BIT_DOWN,
};
#define POOM_BLE_REMOTE_BUTTON_COUNT \
    (sizeof(s_poom_ble_remote_wire_bit) / sizeof(s_poom_ble_remote_wire_bit[0]))

// Readout letters, in wire bit order
static const char s_poom_ble_remote_bit_letter[POOM_BLE_REMOTE_BIT_COUNT] = {
    'U', 'D', 'L', 'R', 'A', 'B',
};

#define POOM_BLE_REMOTE_EXIT_COMBO \
    ((uint16_t)((1U << POOM_BLE_REMOTE_BIT_A) | (1U << POOM_BLE_REMOTE_BIT_B)))

// Static variables
static TaskHandle_t s_poom_ble_remote_task = NULL;
static bool s_poom_ble_remote_running = false;
static bool s_poom_ble_remote_buttons_subscribed = false;
static bool s_poom_ble_remote_gatt_up = false;
static char s_poom_ble_remote_sbus_user[] = "poom_ble_remote";
static poom_ble_remote_exit_cb_t s_poom_ble_remote_exit_cb;
static void *s_poom_ble_remote_exit_cb_ctx;
// Held buttons in wire bit order, written from the SBUS dispatcher task and
// read from the app task.
static volatile uint16_t s_poom_ble_remote_held = 0U;
static uint8_t s_poom_ble_remote_seq = 0U;
static uint16_t s_poom_ble_remote_last_sent = POOM_BLE_REMOTE_NO_REPORT_SENT;
// LED strip: owned by this app while it runs (taken over from the rainbow
// animation, handed back on exit)
static ws2812_strip_t s_poom_ble_remote_strip;
static bool s_poom_ble_remote_leds_ready = false;
static bool s_poom_ble_remote_rainbow_was_running = false;
static bool s_poom_ble_remote_flash = false;

// Function prototypes
static esp_err_t poom_ble_remote_start(void);
static void poom_ble_remote_exit_cb(void *user_ctx);
static void poom_ble_remote_set_exit_callback(poom_ble_remote_exit_cb_t callback, void *user_ctx);
static void poom_ble_remote_button_cb(const poom_sbus_msg_t *msg, void *user_ctx);
static void poom_ble_remote_connection_cb(bool connected, const uint8_t *bda, void *user_ctx);
static void poom_ble_remote_task(void *parameters);
static void poom_ble_remote_cleanup(void *parameters);
static void poom_ble_remote_reset(void);
static uint32_t poom_ble_remote_now_ms(void);
static bool poom_ble_remote_send_report(uint16_t buttons);
static void poom_ble_remote_draw(uint16_t held);
static void poom_ble_remote_leds_acquire(void);
static void poom_ble_remote_leds_release(void);
static void poom_ble_remote_update_leds(uint32_t now_ms);

// Entry point
static esp_err_t poom_ble_remote_start(void)
{
    poom_ble_remote_gatt_cbs_t cbs = {
        .on_connection = poom_ble_remote_connection_cb,
        .on_status_write = NULL,
        .user_ctx = NULL,
    };

    if (s_poom_ble_remote_running)
    {
        return ESP_ERR_INVALID_STATE;
    }

    POOM_BLE_REMOTE_PRINTF_D("Entering poom_ble_remote_start");
    poom_ble_remote_reset();

    if (!poom_sbus_subscribe_cb("input/button", poom_ble_remote_button_cb, s_poom_ble_remote_sbus_user))
    {
        return ESP_FAIL;
    }
    s_poom_ble_remote_buttons_subscribed = true;

    if (poom_ble_remote_gatt_start(&cbs) != ESP_OK)
    {
        (void)poom_sbus_unsubscribe_cb("input/button", poom_ble_remote_button_cb, s_poom_ble_remote_sbus_user);
        s_poom_ble_remote_buttons_subscribed = false;
        return ESP_FAIL;
    }
    s_poom_ble_remote_gatt_up = true;

    if (xTaskCreate(poom_ble_remote_task,
                    "poom_ble_remote",
                    POOM_BLE_REMOTE_TASK_STACK,
                    NULL,
                    POOM_BLE_REMOTE_TASK_PRIO,
                    &s_poom_ble_remote_task) != pdPASS)
    {
        poom_ble_remote_gatt_stop();
        s_poom_ble_remote_gatt_up = false;
        (void)poom_sbus_unsubscribe_cb("input/button", poom_ble_remote_button_cb, s_poom_ble_remote_sbus_user);
        s_poom_ble_remote_buttons_subscribed = false;
        s_poom_ble_remote_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    POOM_BLE_REMOTE_PRINTF_D("Leaving poom_ble_remote_start");
    s_poom_ble_remote_running = true;
    return ESP_OK;
}

static void poom_ble_remote_task(void *parameters)
{
    uint32_t last_notify_ms = 0U;
    uint32_t last_draw_ms = 0U;
    uint32_t combo_held_since_ms = 0U;

    poom_ble_remote_leds_acquire();

    for (;;)
    {
        const uint32_t now = poom_ble_remote_now_ms();
        const uint16_t held = s_poom_ble_remote_held;

        // Exit on A+B held together
        if ((held & POOM_BLE_REMOTE_EXIT_COMBO) == POOM_BLE_REMOTE_EXIT_COMBO)
        {
            if (combo_held_since_ms == 0U)
            {
                combo_held_since_ms = now;
            }
            else if ((now - combo_held_since_ms) >= POOM_BLE_REMOTE_EXIT_HOLD_MS)
            {
                break;
            }
        }
        else
        {
            combo_held_since_ms = 0U;
        }

        if (poom_ble_remote_gatt_notifications_enabled())
        {
            const bool changed = (held != s_poom_ble_remote_last_sent);

            if (changed || ((now - last_notify_ms) >= POOM_BLE_REMOTE_HEARTBEAT_MS))
            {
                if (poom_ble_remote_send_report(held))
                {
                    last_notify_ms = now;
                    s_poom_ble_remote_flash = s_poom_ble_remote_flash || changed;
                }
            }
        }
        else
        {
            // Nobody is listening. Forget what was sent so the first report
            // after a fresh subscription carries the current state.
            s_poom_ble_remote_last_sent = POOM_BLE_REMOTE_NO_REPORT_SENT;
            last_notify_ms = now;
        }

        if ((now - last_draw_ms) >= POOM_BLE_REMOTE_DRAW_MS)
        {
            last_draw_ms = now;
            poom_ble_remote_draw(held);
            poom_ble_remote_update_leds(now);
            s_poom_ble_remote_flash = false;
        }

        vTaskDelay(pdMS_TO_TICKS(POOM_BLE_REMOTE_TICK_MS));
    }

    // Tell the robot to stop now instead of making it wait out the Pi's 300 ms
    // heartbeat timeout. A and B are still held at this point, so the report
    // has to say "nothing pressed" explicitly.
    if (poom_ble_remote_gatt_notifications_enabled())
    {
        (void)poom_ble_remote_send_report(0U);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    poom_ble_remote_cleanup(parameters);
    vTaskDelete(NULL);
}

static void poom_ble_remote_cleanup(void *parameters)
{
    poom_ble_remote_exit_cb_t exit_cb = s_poom_ble_remote_exit_cb;
    void *exit_ctx = s_poom_ble_remote_exit_cb_ctx;

    (void)parameters;

    POOM_BLE_REMOTE_PRINTF_D("Entering poom_ble_remote_cleanup");

    if (s_poom_ble_remote_gatt_up)
    {
        poom_ble_remote_gatt_stop();
        s_poom_ble_remote_gatt_up = false;
    }

    if (s_poom_ble_remote_buttons_subscribed)
    {
        (void)poom_sbus_unsubscribe_cb("input/button", poom_ble_remote_button_cb, s_poom_ble_remote_sbus_user);
        s_poom_ble_remote_buttons_subscribed = false;
    }

    poom_ble_remote_leds_release();

    s_poom_ble_remote_running = false;

    if (exit_cb != NULL)
    {
        exit_cb(exit_ctx);
    }
}

static uint32_t poom_ble_remote_now_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

/**
 * @brief Encodes and notifies one report, advancing the sequence number.
 *
 * @param[in] buttons Button bitmask to report.
 * @return bool true when the notification was queued.
 */
static bool poom_ble_remote_send_report(uint16_t buttons)
{
    uint8_t report[POOM_BLE_REMOTE_REPORT_LEN];

    // The Poom has no battery sensing, so the battery byte is always "unknown".
    poom_ble_remote_encode_report(report,
                                  s_poom_ble_remote_seq,
                                  buttons,
                                  POOM_BLE_REMOTE_BATTERY_UNKNOWN);

    if (poom_ble_remote_gatt_notify_buttons(report, sizeof(report)) != ESP_OK)
    {
        return false;
    }

    s_poom_ble_remote_seq++;
    s_poom_ble_remote_last_sent = buttons;
    return true;
}

static void poom_ble_remote_button_cb(const poom_sbus_msg_t *msg, void *user_ctx)
{
    button_event_msg_t ev;
    uint16_t bit;

    (void)user_ctx;

    if ((msg == NULL) || (msg->len < sizeof(ev)))
    {
        return;
    }

    (void)memcpy(&ev, msg->data, sizeof(ev));

    if (ev.button >= POOM_BLE_REMOTE_BUTTON_COUNT)
    {
        return;
    }

    bit = (uint16_t)(1U << s_poom_ble_remote_wire_bit[ev.button]);

    // Held state only. The driver also publishes click and long-press events
    // for the same physical press; acting on those too would double-count.
    if (ev.event == BUTTON_PRESS_DOWN)
    {
        s_poom_ble_remote_held |= bit;
    }
    else if (ev.event == BUTTON_PRESS_UP)
    {
        s_poom_ble_remote_held &= (uint16_t)~bit;
    }
    else
    {
        // nothing to do
    }
}

static void poom_ble_remote_connection_cb(bool connected, const uint8_t *bda, void *user_ctx)
{
    (void)bda;
    (void)user_ctx;

    if (!connected)
    {
        // A button held through a dropout must not come back as still pressed
        // when the Pi reconnects.
        s_poom_ble_remote_held = 0U;
    }
}

static void poom_ble_remote_exit_cb(void *user_ctx)
{
    const uint8_t token = 1U;

    (void)user_ctx;

    (void)poom_sbus_publish(POOM_MENU_RESUME_TOPIC, &token, sizeof(token), 0);
}

static void poom_ble_remote_set_exit_callback(poom_ble_remote_exit_cb_t callback, void *user_ctx)
{
    s_poom_ble_remote_exit_cb = callback;
    s_poom_ble_remote_exit_cb_ctx = user_ctx;
}

/// @brief Entry point for the poom menu
///
void app_poom_ble_remote_menu(void)
{
    poom_ble_remote_set_exit_callback(poom_ble_remote_exit_cb, NULL);

    // The menu detaches from input/button before launching and only
    // re-attaches on poom/menu/resume; if start fails the app task will
    // never publish it, so do it here or all input stays dead.
    if (poom_ble_remote_start() != ESP_OK)
    {
        poom_ble_remote_exit_cb(NULL);
    }
}

// reset all flags and state variables to their default state
static void poom_ble_remote_reset(void)
{
    s_poom_ble_remote_task = NULL;
    s_poom_ble_remote_running = false;
    s_poom_ble_remote_buttons_subscribed = false;
    s_poom_ble_remote_gatt_up = false;
    s_poom_ble_remote_held = 0U;
    s_poom_ble_remote_seq = 0U;
    s_poom_ble_remote_last_sent = POOM_BLE_REMOTE_NO_REPORT_SENT;
    s_poom_ble_remote_leds_ready = false;
    s_poom_ble_remote_rainbow_was_running = false;
    s_poom_ble_remote_flash = false;
}

// Draws the status screen: link state, peer, report counter and the live
// button readout.
static void poom_ble_remote_draw(uint16_t held)
{
    char line[22];
    char readout[POOM_BLE_REMOTE_BIT_COUNT + 1];
    uint8_t peer[6];
    const bool connected = poom_ble_remote_gatt_is_connected();
    const bool subscribed = poom_ble_remote_gatt_notifications_enabled();

    poom_arduboy_clear();
    poom_arduboy_set_text_size(1);

    // header bar
    poom_arduboy_set_cursor(28, 2);
    (void)poom_arduboy_print("BLE REMOTE");
    poom_arduboy_fill_rect(0, 0, ARDUBOY_WIDTH, 11, INVERT);

    // link state
    (void)snprintf(line, sizeof(line), "LINK: %s", connected ? "CONNECTED" : "ADVERTISING");
    poom_arduboy_set_cursor(4, 16);
    (void)poom_arduboy_print(line);

    // peer, or the name the Pi scans for while there is none
    if (connected && poom_ble_remote_gatt_peer_address(peer))
    {
        (void)snprintf(line, sizeof(line), "%02X:%02X:%02X:%02X:%02X:%02X",
                       peer[0], peer[1], peer[2], peer[3], peer[4], peer[5]);
    }
    else
    {
        (void)snprintf(line, sizeof(line), "AS %s", POOM_BLE_REMOTE_DEVICE_NAME);
    }
    poom_arduboy_set_cursor(4, 26);
    (void)poom_arduboy_print(line);

    // sequence number and subscription state
    (void)snprintf(line, sizeof(line), "SEQ:%-4u NTF:%s",
                   (unsigned)s_poom_ble_remote_seq, subscribed ? "ON" : "OFF");
    poom_arduboy_set_cursor(4, 36);
    (void)poom_arduboy_print(line);

    // live button readout, one fixed column per button
    for (uint8_t i = 0U; i < (uint8_t)POOM_BLE_REMOTE_BIT_COUNT; i++)
    {
        readout[i] = ((held & (1U << i)) != 0U) ? s_poom_ble_remote_bit_letter[i] : '.';
    }
    readout[POOM_BLE_REMOTE_BIT_COUNT] = '\0';
    (void)snprintf(line, sizeof(line), "BTN: %s", readout);
    poom_arduboy_set_cursor(4, 46);
    (void)poom_arduboy_print(line);

    // footer
    poom_arduboy_set_cursor(0, 56);
    (void)poom_arduboy_print("  A+B HOLD: EXIT");

    poom_arduboy_display();
}

static void poom_ble_remote_leds_acquire(void)
{
    s_poom_ble_remote_rainbow_was_running = poom_led_rainbow_deinit();

    if (ws2812_init(&s_poom_ble_remote_strip,
                    PIN_NUM_WS2812,
                    PIN_NUM_LEDS,
                    false,
                    POOM_BLE_REMOTE_LED_RESOLUTION_HZ) != ESP_OK)
    {
        POOM_BLE_REMOTE_PRINTF_D("LED init failed, running screen-only");
        poom_led_rainbow_init();
        if (s_poom_ble_remote_rainbow_was_running)
        {
            (void)poom_led_rainbow_start();
        }
        s_poom_ble_remote_rainbow_was_running = false;
        s_poom_ble_remote_leds_ready = false;
        return;
    }

    ws2812_set_brightness(&s_poom_ble_remote_strip, POOM_BLE_REMOTE_LED_BRIGHTNESS);
    ws2812_clear(&s_poom_ble_remote_strip);
    (void)ws2812_show(&s_poom_ble_remote_strip);
    s_poom_ble_remote_leds_ready = true;
}

static void poom_ble_remote_leds_release(void)
{
    if (!s_poom_ble_remote_leds_ready)
    {
        // acquire failed and already restored the rainbow
        return;
    }

    ws2812_clear(&s_poom_ble_remote_strip);
    (void)ws2812_show(&s_poom_ble_remote_strip);
    ws2812_deinit(&s_poom_ble_remote_strip);
    s_poom_ble_remote_leds_ready = false;

    poom_led_rainbow_init();
    if (s_poom_ble_remote_rainbow_was_running)
    {
        (void)poom_led_rainbow_start();
    }
    s_poom_ble_remote_rainbow_was_running = false;
}

// Amber while nobody is listening, green once the Pi has subscribed, and the
// first LED flashes white on every report that carries a button change.
static void poom_ble_remote_update_leds(uint32_t now_ms)
{
    uint8_t r = 255U;
    uint8_t g = 150U;
    uint8_t b = 0U;

    if (!s_poom_ble_remote_leds_ready)
    {
        return;
    }

    if (poom_ble_remote_gatt_notifications_enabled())
    {
        r = 0U;
        g = 200U;
        b = 0U;
    }
    else if (!poom_ble_remote_gatt_is_connected() &&
             (((now_ms / POOM_BLE_REMOTE_LED_BLINK_MS) % 2U) != 0U))
    {
        // blink while still advertising
        r = 0U;
        g = 0U;
    }
    else
    {
        // connected but not subscribed yet: solid amber
    }

    ws2812_fill(&s_poom_ble_remote_strip, r, g, b, 0U);

    if (s_poom_ble_remote_flash)
    {
        ws2812_set_pixel(&s_poom_ble_remote_strip, 0, 255U, 255U, 255U, 0U);
    }

    (void)ws2812_show(&s_poom_ble_remote_strip);
}
