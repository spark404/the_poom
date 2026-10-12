set(srcs "poom_ble_remote.c" "poom_ble_remote_gatt.c")

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS "include"
    REQUIRES bt nvs_flash button_driver poom_arduboy_display poom_sbus ws2812 poom_led_rainbow board
)
