#include <stdio.h>
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

// motor driver: pololu DRV8838

/*
runs motor at constant half speed in forward direction
*/

/*
wiring:
GND --- battery/power ground
    make sure MCU GND connected to same ground
VM or VIN (same thing) --- batter/power power
O1 --- one lead of DC motor
O2 --- other lead of DC motor

VCC --- MCU 3.3/5V pin
PH --- direction (pin 19)
EN --- PWM (pin 18)
(low)SLP --- set HIGH to keep driver active (can just tie to VCC)
                or connect to pin 21 (nsleep pin)
*/

#define MOTOR_PWM_GPIO          18
#define MOTOR_DIR_GPIO          19
#define MOTOR_NSLEEP_GPIO       21

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL            LEDC_CHANNEL_0

// set duty cycle resolution; 4096/8191 ~= 50% speed
// set hardware timer to 4kHz
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT   // Resolution: 0 to 8191
#define LEDC_FREQUENCY          4000                // 4 kHz frequency
#define LEDC_DUTY               4096                // 50% speed


void app_main(void)
{

    // set direction pin
    // set to high to spin 'forward' (depends on O1, O2 wiring)
    gpio_reset_pin(MOTOR_DIR_GPIO);
    gpio_set_direction(MOTOR_DIR_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_DIR_GPIO,1);

    // set nsleep pin
    // set to high to wake/enable driver
    gpio_reset_pin(MOTOR_NSLEEP_GPIO);
    gpio_set_direction(MOTOR_NSLEEP_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_NSLEEP_GPIO,1);

    // setup pwm hardware timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode                 = LEDC_MODE,
        .timer_num                  = LEDC_TIMER,
        .duty_resolution            = LEDC_DUTY_RES,
        .freq_hz                    = LEDC_FREQUENCY,
        .clk_cfg                    = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // bind timer through channel to pwm pin
    ledc_channel_config_t ledc_channel = {
        .speed_mode                 = LEDC_MODE,
        .channel                    = LEDC_CHANNEL,
        .timer_sel                  = LEDC_TIMER,
        .gpio_num                   = MOTOR_PWM_GPIO,
        .duty                       = 0,
        .hpoint                     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // set the ledc duty
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));

    // commit the ledc duty, begins outputting pwm signal
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}
