#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "hal/gpio_types.h"

#define TAG "TCS230"

#define S0_PIN 12
#define S1_PIN 14
#define S2_PIN 27
#define S3_PIN 26
#define OUT_PIN 25

#define PCNT_L_LIM -500
#define PCNT_H_LIM 500

#define R_RED 4220
#define G_RED 3560
#define B_RED 4080
#define C_RED 1880

#define R_GREEN 3520
#define G_GREEN 3520
#define B_GREEN 3680
#define C_GREEN 840

static void select_photodiode_type(const char *colour)
{
    // ESP_LOGI(TAG, "Select colour: %s", colour);
    if (strcmp(colour, "RED") == 0) {
        gpio_set_level(S2_PIN, 0);
        gpio_set_level(S3_PIN, 0);
    } else if (strcmp(colour, "GREEN") == 0) {
        gpio_set_level(S2_PIN, 1);
        gpio_set_level(S3_PIN, 1);
    } else if (strcmp(colour, "BLUE") == 0) {
        gpio_set_level(S2_PIN, 0);
        gpio_set_level(S3_PIN, 1);
    } else if (strcmp(colour, "CLEAR") == 0) {
        gpio_set_level(S2_PIN, 1);
        gpio_set_level(S3_PIN, 0);
    } else {
        ESP_LOGW(TAG, "Unknown color selection: %s", colour);
    }
}

static int read_frequency_hz(pcnt_unit_handle_t *pcnt_unit)
{
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(*pcnt_unit));
    vTaskDelay(50/portTICK_PERIOD_MS);   // 50ms measurement window
    ESP_ERROR_CHECK(pcnt_unit_stop(*pcnt_unit));

    int pulse_count = 0;
    // for (int i = 0; i < 5; i++) {
    //     int count = 0;
    //     ESP_ERROR_CHECK(pcnt_unit_get_count(*pcnt_unit, &count));
    //     pulse_count += count;
    //     ESP_ERROR_CHECK(pcnt_unit_clear_count(*pcnt_unit));
    //     ESP_LOGI(TAG, "Sample %d: %d pulses", i+1, pulse_count);
    //     vTaskDelay(1000/portTICK_PERIOD_MS);
    // }
    // ESP_ERROR_CHECK(pcnt_unit_stop(*pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_get_count(*pcnt_unit, &pulse_count));

    // Convert to Hz
    return pulse_count * 20;   // 50ms window = multiply by 20
    // return 100;
    // return pulse_count;
}

static int read_colour(const char *colour, pcnt_unit_handle_t *pcnt_unit)
{
    select_photodiode_type(colour);
    vTaskDelay(20/portTICK_PERIOD_MS);
    return read_frequency_hz(pcnt_unit);
}

static char* evaluate_colour(int red, int green, int blue)
{
    if (abs(red - R_RED) < 500 &&
        abs(green - G_RED) < 500 &&
        abs(blue - B_RED) < 500) {
        return "RED";
    } else if (abs(red - R_GREEN) < 500 &&
               abs(green - G_GREEN) < 500 &&
               abs(blue - B_GREEN) < 500) {
        return "GREEN";
    } else {
        return "UNKNOWN";
    }
}

void app_main(void)
{

    // configure pins
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 
            (1ULL << S0_PIN) | (1ULL << S1_PIN) |
            (1ULL << S2_PIN) | (1ULL << S3_PIN)
    };
    gpio_config(&io_conf);

    gpio_set_direction(OUT_PIN, GPIO_MODE_INPUT);

    // while (1) {
    //     int level = gpio_get_level(OUT_PIN);
    //     ESP_LOGI("GPIO", "Pin %d level: %d", OUT_PIN, level);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // set scaling frequency: S0=1, S1=0 -> 20% scaling
    gpio_set_level(S0_PIN, 1);
    gpio_set_level(S1_PIN, 0);

    // configure pcnt
    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_L_LIM,
        .high_limit = PCNT_H_LIM,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = OUT_PIN,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD)
    );

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));

    ESP_LOGI(TAG, "TCS230 Colour Sensor Example Running...");


    int red, green, blue, clear;
    char* detected_colour;
    while (1) {
        red = read_colour("RED", &pcnt_unit);
        green = read_colour("GREEN", &pcnt_unit);
        blue = read_colour("BLUE", &pcnt_unit);
        clear = read_colour("CLEAR", &pcnt_unit);
        detected_colour = evaluate_colour(red, green, blue);

        ESP_LOGI(TAG, "R=%d  G=%d  B=%d C=%d", red, green, blue, clear);
        ESP_LOGI(TAG, "Detected Colour: %s", detected_colour);

        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}
