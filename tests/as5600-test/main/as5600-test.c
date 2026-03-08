/**
 * AS5600 Magnetic Rotary Encoder - ESP-IDF Test Program
 *
 * Wiring:
 *   AS5600 VCC  -> ESP32 3.3V
 *   AS5600 GND  -> ESP32 GND
 *   AS5600 SDA  -> ESP32 GPIO 21
 *   AS5600 SCL  -> ESP32 GPIO 22
 *   AS5600 DIR  -> ESP32 GND (CW = increasing angle) or 3.3V (CCW = increasing)
 *   AS5600 OUT  -> Not used in this test (analog/PWM output)
 *   AS5600 GPO  -> Not used in this test (power indicator)
 *
 * This test program:
 *   1. Verifies I2C communication with the AS5600
 *   2. Reads and prints all configuration registers
 *   3. Checks magnet detection status
 *   4. Continuously reads the raw angle and computed angle
 *   5. Detects rotation direction and speed (RPM)
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

/* ── I2C Configuration ────────────────────────────────────────────── */
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_FREQ_HZ      400000   /* 400 kHz Fast Mode */
#define I2C_MASTER_TIMEOUT_MS   100

/* ── AS5600 I2C Address & Registers ──────────────────────────────── */
#define AS5600_I2C_ADDR         0x36

/* Configuration Registers */
#define AS5600_REG_ZMCO         0x00  /* Number of times ZPOS/MPOS burned */
#define AS5600_REG_ZPOS_H       0x01  /* Zero Position (MSB) */
#define AS5600_REG_ZPOS_L       0x02  /* Zero Position (LSB) */
#define AS5600_REG_MPOS_H       0x03  /* Maximum Position (MSB) */
#define AS5600_REG_MPOS_L       0x04  /* Maximum Position (LSB) */
#define AS5600_REG_MANG_H       0x05  /* Maximum Angle (MSB) */
#define AS5600_REG_MANG_L       0x06  /* Maximum Angle (LSB) */
#define AS5600_REG_CONF_H       0x07  /* Configuration (MSB) */
#define AS5600_REG_CONF_L       0x08  /* Configuration (LSB) */

/* Output Registers */
#define AS5600_REG_RAWANGLE_H   0x0C  /* Raw Angle (MSB) */
#define AS5600_REG_RAWANGLE_L   0x0D  /* Raw Angle (LSB) */
#define AS5600_REG_ANGLE_H      0x0E  /* Angle (MSB) - filtered, with start/stop */
#define AS5600_REG_ANGLE_L      0x0F  /* Angle (LSB) */

/* Status Registers */
#define AS5600_REG_STATUS       0x0B  /* Magnet detection status */
#define AS5600_REG_AGC          0x1A  /* Automatic Gain Control */
#define AS5600_REG_MAGNITUDE_H  0x1B  /* CORDIC Magnitude (MSB) */
#define AS5600_REG_MAGNITUDE_L  0x1C  /* CORDIC Magnitude (LSB) */

/* Burn Command Register */
#define AS5600_REG_BURN         0xFF

/* STATUS register bit masks */
#define AS5600_STATUS_MD        (1 << 5)  /* Magnet Detected */
#define AS5600_STATUS_ML        (1 << 4)  /* Magnet Too Weak (Low) */
#define AS5600_STATUS_MH        (1 << 3)  /* Magnet Too Strong (High) */

/* ── Helpers ──────────────────────────────────────────────────────── */
#define AS5600_RAW_MAX          4095.0f   /* 12-bit resolution */
#define DEGREES_PER_RAW         (360.0f / 4096.0f)

static const char *TAG = "AS5600";

/* ── I2C Driver Init ─────────────────────────────────────────────── */
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    return i2c_driver_install(
        I2C_MASTER_NUM, 
        conf.mode, 
        0, 
        0, 
        0
    );
}

/* ── AS5600 Register Read ────────────────────────────────────────── */
static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM,
        AS5600_I2C_ADDR,
        &reg, 1,
        data, len,
        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS)
    );
}

/* ── Read a 12-bit value from two consecutive registers ─────────── */
static esp_err_t as5600_read_u16(uint8_t reg_h, uint16_t *out)
{
    uint8_t buf[2];
    esp_err_t err = as5600_read_reg(reg_h, buf, 2);
    if (err == ESP_OK) {
        *out = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    }
    return err;
}

/* ── Test 1: I2C Connectivity ────────────────────────────────────── */
static void test_i2c_connection(void)
{
    printf("\n========================================\n");
    printf("  TEST 1: I2C Connectivity\n");
    printf("========================================\n");

    uint8_t dummy;
    esp_err_t err = as5600_read_reg(AS5600_REG_STATUS, &dummy, 1);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[PASS] AS5600 found at I2C address 0x%02X", AS5600_I2C_ADDR);
    } else {
        ESP_LOGE(TAG, "[FAIL] AS5600 not found! Error: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "       Check wiring: SDA->GPIO%d, SCL->GPIO%d, VCC->3.3V",
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    }
}

/* ── Test 2: Magnet Detection ────────────────────────────────────── */
static bool test_magnet_detection(void)
{
    printf("\n========================================\n");
    printf("  TEST 2: Magnet Detection\n");
    printf("========================================\n");

    uint8_t status;
    esp_err_t err = as5600_read_reg(AS5600_REG_STATUS, &status, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[FAIL] Could not read STATUS register: %s", esp_err_to_name(err));
        return false;
    }

    uint8_t agc;
    as5600_read_reg(AS5600_REG_AGC, &agc, 1);

    uint16_t magnitude;
    uint8_t mag_buf[2];
    as5600_read_reg(AS5600_REG_MAGNITUDE_H, mag_buf, 2);
    magnitude = ((uint16_t)(mag_buf[0] & 0x0F) << 8) | mag_buf[1];

    printf("  STATUS raw  : 0x%02X\n", status);
    printf("  AGC value   : %u  (0=strong, 255=weak)\n", agc);
    printf("  Magnitude   : %u\n", magnitude);

    bool md = (status & AS5600_STATUS_MD) != 0;
    bool ml = (status & AS5600_STATUS_ML) != 0;
    bool mh = (status & AS5600_STATUS_MH) != 0;

    printf("  Magnet Detected (MD) : %s\n", md ? "YES" : "NO");
    printf("  Magnet Too Weak (ML) : %s\n", ml ? "YES ⚠" : "no");
    printf("  Magnet Too Strong(MH): %s\n", mh ? "YES ⚠" : "no");

    if (md && !ml && !mh) {
        ESP_LOGI(TAG, "[PASS] Magnet detected and field strength is optimal");
        return true;
    } else if (!md) {
        ESP_LOGE(TAG, "[FAIL] No magnet detected! Place a diametrically magnetised magnet above the IC");
        return false;
    } else if (ml) {
        ESP_LOGW(TAG, "[WARN] Magnet too weak or too far — move magnet closer");
        return true;
    } else if (mh) {
        ESP_LOGW(TAG, "[WARN] Magnet too strong or too close — move magnet further away");
        return true;
    }
    return false;
}

/* ── Test 3: Configuration Registers ────────────────────────────── */
static void test_read_config(void)
{
    printf("\n========================================\n");
    printf("  TEST 3: Configuration Registers\n");
    printf("========================================\n");

    uint8_t zmco;
    as5600_read_reg(AS5600_REG_ZMCO, &zmco, 1);

    uint16_t zpos, mpos, mang, conf;
    as5600_read_u16(AS5600_REG_ZPOS_H, &zpos);
    as5600_read_u16(AS5600_REG_MPOS_H, &mpos);
    as5600_read_u16(AS5600_REG_MANG_H, &mang);
    as5600_read_u16(AS5600_REG_CONF_H, &conf);

    /* Decode CONF register fields */
    uint8_t pm    = (conf >> 0) & 0x03;  /* Power Mode */
    uint8_t hyst  = (conf >> 2) & 0x03;  /* Hysteresis */
    uint8_t outs  = (conf >> 4) & 0x03;  /* Output Stage */
    uint8_t pwmf  = (conf >> 6) & 0x03;  /* PWM Frequency */
    uint8_t sf    = (conf >> 8) & 0x03;  /* Slow Filter */
    uint8_t fth   = (conf >> 10) & 0x07; /* Fast Filter Threshold */
    uint8_t wd    = (conf >> 13) & 0x01; /* Watchdog */

    const char *pm_str[]   = {"NOM", "LPM1", "LPM2", "LPM3"};
    const char *hyst_str[] = {"OFF", "1 LSB", "2 LSBs", "3 LSBs"};
    const char *outs_str[] = {"Analog (0-100%)", "Analog (10-90%)", "Digital PWM", "Reserved"};
    const char *pwmf_str[] = {"115 Hz", "230 Hz", "460 Hz", "920 Hz"};
    const char *sf_str[]   = {"16x", "8x", "4x", "2x"};

    printf("  ZMCO (burn count)  : %u / 3\n", zmco);
    printf("  ZPOS (zero angle)  : %u (%.2f°)\n", zpos, zpos * DEGREES_PER_RAW);
    printf("  MPOS (max angle)   : %u (%.2f°)\n", mpos, mpos * DEGREES_PER_RAW);
    printf("  MANG (angle range) : %u (%.2f°)\n", mang, mang * DEGREES_PER_RAW);
    printf("  CONF raw           : 0x%04X\n", conf);
    printf("    Power Mode       : %s\n", pm_str[pm]);
    printf("    Hysteresis       : %s\n", hyst_str[hyst]);
    printf("    Output Stage     : %s\n", outs_str[outs]);
    printf("    PWM Frequency    : %s\n", pwmf_str[pwmf]);
    printf("    Slow Filter      : %s\n", sf_str[sf]);
    printf("    Fast Filter Thr  : %u\n", fth);
    printf("    Watchdog         : %s\n", wd ? "ON" : "OFF");

    ESP_LOGI(TAG, "[PASS] Configuration registers read successfully");
}

/* ── Test 4: Continuous Angle Reading ───────────────────────────── */
static void test_continuous_angle(void)
{
    printf("\n========================================\n");
    printf("  TEST 4: Continuous Angle Reading\n");
    printf("  Rotate the magnet to verify output.\n");
    printf("  Press Ctrl+C (reset) to stop.\n");
    printf("========================================\n");

    const int SAMPLES          = 200;
    const int SAMPLE_MS        = 50;   /* 20 Hz */
    const float RPM_ALPHA      = 0.1f; /* EMA smoothing */

    uint16_t prev_raw  = 0;
    float    rpm_ema   = 0.0f;
    bool     first     = true;
    int64_t  prev_time = esp_timer_get_time();

    for (int i = 0; i < SAMPLES; i++) {

        uint16_t raw_angle, angle;
        esp_err_t err1 = as5600_read_u16(AS5600_REG_RAWANGLE_H, &raw_angle);
        esp_err_t err2 = as5600_read_u16(AS5600_REG_ANGLE_H,    &angle);

        if (err1 != ESP_OK || err2 != ESP_OK) {
            ESP_LOGE(TAG, "Read error at sample %d", i);
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
            continue;
        }

        int64_t now   = esp_timer_get_time();
        float   dt_s  = (now - prev_time) / 1e6f;
        prev_time     = now;

        float degrees_raw = raw_angle * DEGREES_PER_RAW;
        float degrees     = angle     * DEGREES_PER_RAW;

        /* RPM calculation via delta angle */
        if (!first) {
            int16_t delta = (int16_t)raw_angle - (int16_t)prev_raw;

            /* Handle 12-bit wrap-around */
            if (delta >  2048) delta -= 4096;
            if (delta < -2048) delta += 4096;

            float delta_deg = delta * DEGREES_PER_RAW;
            float rpm_inst  = (delta_deg / 360.0f) / dt_s * 60.0f;
            rpm_ema = RPM_ALPHA * rpm_inst + (1.0f - RPM_ALPHA) * rpm_ema;
        }
        first    = false;
        prev_raw = raw_angle;

        /* Direction indicator */
        const char *dir = (rpm_ema > 0.5f)  ? "CW  →" :
                          (rpm_ema < -0.5f) ? "← CCW" : "STILL";

        printf("  [%3d] Raw: %4u (%7.3f°) | Angle: %4u (%7.3f°) | RPM: %+7.2f | %s\n",
               i, raw_angle, degrees_raw, angle, degrees, rpm_ema, dir);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }

    ESP_LOGI(TAG, "[PASS] Continuous angle test complete");
}

/* ── App Entry Point ─────────────────────────────────────────────── */
void app_main(void)
{
    printf("\n");
    printf("╔══════════════════════════════════════╗\n");
    printf("║   AS5600 Encoder Functional Test     ║\n");
    printf("║   ESP-IDF                            ║\n");
    printf("╚══════════════════════════════════════╝\n");
    printf("  SDA -> GPIO%d\n", I2C_MASTER_SDA_IO);
    printf("  SCL -> GPIO%d\n", I2C_MASTER_SCL_IO);
    printf("  I2C addr: 0x%02X\n\n", AS5600_I2C_ADDR);

    /* Initialise I2C */
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C master initialised at %d Hz", I2C_MASTER_FREQ_HZ);

    /* Small delay for sensor power-on */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Run tests */
    test_i2c_connection();
    bool magnet_ok = test_magnet_detection();
    test_read_config();

    if (magnet_ok) {
        test_continuous_angle();
    } else {
        ESP_LOGE(TAG, "Skipping angle test — fix magnet placement first");
    }

    printf("\n========================================\n");
    printf("  All tests finished.\n");
    printf("========================================\n");

    /* Loop forever — re-run angle test every 5 s */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        magnet_ok = test_magnet_detection();
        if (magnet_ok) {
            test_continuous_angle();
        }
    }
}