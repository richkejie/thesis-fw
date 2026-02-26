#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "hal/pcnt_types.h"

/*
wiring:
GND --- MCU GND
VCC --- MCU 3.3/5V pin
C1 --- GPIO pin 34
C2 --- GPIO pin 35

M1, M2 --- disconnected (not used to test encoder)
*/

static const char *TAG = "gearmotor-encoder";

#define ENCODER_GPIO_C1         34
#define ENCODER_GPIO_C2         35
#define ENCODER_HIGH_LIMIT      1000
#define ENCODER_LOW_LIMIT       -1000

void app_main(void)
{
    // configure pulse counter unit
    pcnt_unit_config_t unit_config = {
        .high_limit     = ENCODER_HIGH_LIMIT,
        .low_limit      = ENCODER_LOW_LIMIT,
        // .flags.accum_count = true,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // glitch filter to debounce signal
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // c1 edge, c2 level
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num      = ENCODER_GPIO_C1,
        .level_gpio_num     = ENCODER_GPIO_C2,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // set actions
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    // enable the counter
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));

    // clear the counter
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));

    // start the counter
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    ESP_LOGI(TAG, "Encoder ready -  spin the shaft and watch the count");
    
    int count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
        pcnt_unit_get_count(pcnt_unit, &count);
        ESP_LOGI(TAG, "Encoder count: %d", count);
    }
}
