/*
 * spool_home.c
 *
 * Homes a spool/gear-motor system by reeling in string until the cap hits the
 * mechanical stop. Stall is detected by BOTH:
 *   1. Encoder silence  — no pulses for STALL_ENCODER_TIMEOUT_MS (200 ms)
 *   2. Current spike    — ADC reading exceeds STALL_CURRENT_THRESHOLD (3000/4095)
 *
 * Once stall is confirmed, the motor stops and the encoder counter is zeroed.
 *
 * ── Pin Assignments (change in the #defines below) ──────────────────────────
 *   ENC_A          GPIO 18   Quadrature encoder channel A
 *   ENC_B          GPIO 19   Quadrature encoder channel B
 *   MOTOR_PWM      GPIO 25   PWM output → motor driver EN/PWM pin
 *   MOTOR_DIR      GPIO 26   Direction output → motor driver DIR pin
 *   CURRENT_SENSE  GPIO 34   ADC1 channel 6 (current sense, input only)
 *
 * ── Tuning constants ────────────────────────────────────────────────────────
 *   HOMING_DUTY            512 / 1023  (~50 % — slow, safe homing speed)
 *   STALL_ENCODER_TIMEOUT  200 ms
 *   STALL_CURRENT_THRESH   3000 / 4095 (~73 % of full-scale)
 *   Both conditions must be true simultaneously to trigger a stall.
 *
 * ── Build notes ─────────────────────────────────────────────────────────────
 *   Requires ESP-IDF ≥ v5.0.
 *   Add to your CMakeLists.txt:
 *       idf_component_register(SRCS "spool_home.c" INCLUDE_DIRS ".")
 */

#include <stdio.h>
#include <stdlib.h>
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "spool_home";

/* ── Pin definitions ───────────────────────────────────────────────────────── */
#define ENC_A_GPIO          18
#define ENC_B_GPIO          19
#define MOTOR_PWM_GPIO      25
#define MOTOR_DIR_GPIO      26
#define CURRENT_SENSE_GPIO  34      /* Must be an ADC1-capable pin            */
#define CURRENT_SENSE_CHAN  ADC_CHANNEL_6   /* GPIO 34 = ADC1 channel 6       */

/* ── Tuning ────────────────────────────────────────────────────────────────── */
#define HOMING_DUTY             600     /* 0–1023, ~50 % PWM                  */
#define STALL_ENCODER_TIMEOUT_MS 200    /* ms of encoder silence → stall      */
#define STALL_CURRENT_V_THRESHOLD 50    /* ADC raw (0–4095) above → stall     */

#define ENCODER_HIGH_LIMIT      1000
#define ENCODER_LOW_LIMIT       -1000

/* ── LEDC (PWM) config ─────────────────────────────────────────────────────── */
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_10_BIT   /* 10-bit → 0–1023            */
#define LEDC_FREQUENCY_HZ   20000               /* 20 kHz, above audible range */

/* ── Encoder globals ───────────────────────────────────────────────────────── */
static pcnt_unit_handle_t   pcnt_unit   = NULL;
static volatile int64_t     last_pulse_time_us = 0;   /* updated in ISR       */
static int                  encoder_position   = 0;   /* zeroed after homing  */

/* ── ADC handle ────────────────────────────────────────────────────────────── */
static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_adc_cali = false;

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);


/* ═══════════════════════════════════════════════════════════════════════════
 *  PCNT watch-point callback — fires on every edge so we can timestamp it
 * ═════════════════════════════════════════════════════════════════════════ */
static bool IRAM_ATTR pcnt_on_reach(pcnt_unit_handle_t unit,
                                    const pcnt_watch_event_data_t *edata,
                                    void *user_ctx)
{
    last_pulse_time_us = esp_timer_get_time();
    return false; /* no task wakeup needed */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialise the quadrature encoder via PCNT
 * ═════════════════════════════════════════════════════════════════════════ */
static void encoder_init(void)
{
    pcnt_unit_config_t unit_config = {
        .low_limit  = ENCODER_LOW_LIMIT,
        .high_limit =  ENCODER_HIGH_LIMIT,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    /* Glitch filter: ignore pulses shorter than 1 µs */
    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    /* Channel — count on A edges, gate with B for direction */
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num  = ENC_A_GPIO,
        .level_gpio_num = ENC_B_GPIO,
    };
    pcnt_channel_handle_t pcnt_chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    /* Watch-point at ±100 counts just to keep the timestamp callback firing */
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit,  100));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, -100));

    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_on_reach };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    last_pulse_time_us = esp_timer_get_time(); /* seed so we don't false-stall */
    ESP_LOGI(TAG, "Encoder initialised on GPIO %d / %d", ENC_A_GPIO, ENC_B_GPIO);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialise LEDC for motor PWM
 * ═════════════════════════════════════════════════════════════════════════ */
static void motor_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQUENCY_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .gpio_num   = MOTOR_PWM_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    /* DIR pin */
    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_DIR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_cfg));

    ESP_LOGI(TAG, "Motor PWM initialised on GPIO %d, DIR on GPIO %d",
             MOTOR_PWM_GPIO, MOTOR_DIR_GPIO);
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialise ADC for current sensing
 * ═════════════════════════════════════════════════════════════════════════ */
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,   /* 0–3.3 V input range               */
        .bitwidth = ADC_BITWIDTH_12,   /* 12-bit → 0–4095                   */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle,
                                               CURRENT_SENSE_CHAN, &chan_cfg));

    // ADC calibration
    // do_adc_cali = example_adc_calibration_init(ADC_UNIT_1, CURRENT_SENSE_CHAN, ADC_ATTEN_DB_12, &adc_cali_handle);

    ESP_LOGI(TAG, "ADC initialised on channel %d (GPIO %d)",
             CURRENT_SENSE_CHAN, CURRENT_SENSE_GPIO);
}

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Motor helpers
 * ═════════════════════════════════════════════════════════════════════════ */
static void motor_set(uint32_t duty, bool forward)
{
    gpio_set_level(MOTOR_DIR_GPIO, forward ? 1 : 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

static void motor_stop(void)
{
    motor_set(0, true);
    ESP_LOGI(TAG, "Motor stopped");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Stall detection helpers
 * ═════════════════════════════════════════════════════════════════════════ */
static bool encoder_stalled(void)
{
    int64_t now     = esp_timer_get_time();
    int64_t silence = (now - last_pulse_time_us) / 1000; /* µs → ms */

    int count = 0;
    pcnt_unit_get_count(pcnt_unit, &count);
    ESP_LOGI(TAG, "Encoder count: %d", count);

    return (silence >= STALL_ENCODER_TIMEOUT_MS);
}

static bool current_spiked(void)
{
    int raw = 0;
    int voltage = 0;
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, CURRENT_SENSE_CHAN, &raw));
    if (do_adc_cali) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage));
        ESP_LOGI(TAG, "ADC Channel Cali Voltage: %d mV", voltage);
    } else {
        ESP_LOGI(TAG, "ADC Channel raw: %d", raw);
    }
    
    if (do_adc_cali) {
        return (voltage >= STALL_CURRENT_V_THRESHOLD);
    } else {
        return (raw >= STALL_CURRENT_V_THRESHOLD);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Main homing routine
 * ═════════════════════════════════════════════════════════════════════════ */
static void home_spool(void)
{
    ESP_LOGI(TAG, "── Homing started ──────────────────────────────");

    /* Reset encoder counter and pulse timestamp */
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    last_pulse_time_us = esp_timer_get_time();

    /* Start motor reeling in (DIR HIGH = forward) */
    motor_set(HOMING_DUTY, true);
    ESP_LOGI(TAG, "Motor running at duty %d toward stop", HOMING_DUTY);

    /*
     * Poll until BOTH stall conditions are true at the same time.
     * Requiring both prevents false triggers: a noisy current reading alone
     * won't stop the motor, nor will a brief encoder dropout.
     */
    while (true)
    {
        bool enc_stall  = encoder_stalled();
        bool cur_spike  = current_spiked();

        if (enc_stall && cur_spike)
        {
            ESP_LOGI(TAG, "Stall detected — encoder silent + current spike");
            break;
        }

        /* Log partial detections at debug level */
        if (enc_stall)
            ESP_LOGD(TAG, "Encoder silent (waiting for current confirmation)");
        if (cur_spike)
            ESP_LOGD(TAG, "Current spike (waiting for encoder confirmation)");

        vTaskDelay(pdMS_TO_TICKS(100));  /* 10 ms poll interval */
    }

    motor_stop();

    /* ── Zero the encoder ── */
    vTaskDelay(pdMS_TO_TICKS(1000));  /* 10 ms poll interval */
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    encoder_position = 0;

    ESP_LOGI(TAG, "Encoder zeroed. Homing complete.");
    ESP_LOGI(TAG, "────────────────────────────────────────────────");
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Optional: read current encoder position (call from your application)
 * ═════════════════════════════════════════════════════════════════════════ */
int spool_get_position(void)
{
    int raw = 0;
    pcnt_unit_get_count(pcnt_unit, &raw);
    return raw;   /* counts from home (zero) */
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  app_main
 * ═════════════════════════════════════════════════════════════════════════ */
void app_main(void)
{
    ESP_LOGI(TAG, "Spool homing system starting");

    encoder_init();
    motor_pwm_init();
    adc_init();

    /* Run the homing sequence once on boot */
    home_spool();

    /* ── Your application loop goes here ── */
    while (true)
    {
        int pos = spool_get_position();
        ESP_LOGI(TAG, "Encoder position: %d counts from home", pos);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}