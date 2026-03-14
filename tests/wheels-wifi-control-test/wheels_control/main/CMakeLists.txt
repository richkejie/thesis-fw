idf_component_register(
    SRCS "wheels_control.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio
        esp_driver_ledc
        driver
        esp_wifi
        esp_event
        esp_netif
        esp_http_server
        nvs_flash
        esp_timer
)