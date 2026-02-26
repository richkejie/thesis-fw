// NOT WORKING!!!


/*
this program requires the bdc_motor component
in the project directory run:
idf.py add-dependency "espressif/bdc_motor^0.2.1"

source: https://components.espressif.com/components/espressif/bdc_motor/versions/0.2.1/readme

example: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_bdc_speed_control/main/mcpwm_bdc_control_example_main.c
*/


#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bdc_motor.h"
#include "esp_timer.h"
#include "freertos/projdefs.h"

#include "driver/gpio.h"
#include "hal/gpio_types.h"


// motor driver: pololu mp6550

/*
runs motor at const speed (half speed)
*/

/*
wiring:
GND --- battery/power ground
    make sure MCU GND connected to same ground
VIN/VM --- battery/power power
OUT1 --- one lead of DC motor
OUT2 --- other lead of DC motor

V3P3 --- MCU 3.3V pin
IN1 --- pin 18 (controls half of H-bridge)
IN2 --- pin 19 (controls other half of H-bridge)
nSLP --- tie to V3P3 (HIGH to keep driver active)

*/

static const char *TAG = "dc-motor-test";

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              18
#define BDC_MCPWM_GPIO_B              19
#define MOTOR_NSLEEP_GPIO       21

void app_main(void)
{

    // set nsleep pin
    // set to high to wake/enable driver
    gpio_reset_pin(MOTOR_NSLEEP_GPIO);
    gpio_set_direction(MOTOR_NSLEEP_GPIO,GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_NSLEEP_GPIO,1);

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));

    // // Give driver a clean state before setting direction
    // ESP_LOGI(TAG, "Coast motor briefly");
    // ESP_ERROR_CHECK(bdc_motor_coast(motor));
    // vTaskDelay(pdMS_TO_TICKS(100));

    /*
    Forward: IN1 receives PWM signal and gearmotor turns 'forward'; IN2 held low
    Backward: IN2 receives PWM signal and gearmotor turns 'backward'; IN1 held low
    */
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));

    // set speed: BDC_MCPWM_DUTY_TICK_MAX/2 (50% speed)
    ESP_LOGI(TAG, "Set motor speed");
    uint32_t speed = BDC_MCPWM_DUTY_TICK_MAX / 2;
    ESP_ERROR_CHECK(bdc_motor_set_speed(motor, speed));


    // Keep running forever
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    // while (1) {
    //     bdc_motor_forward(motor);
    //     ESP_LOGI(TAG, "Trying forward");
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     bdc_motor_reverse(motor);
    //     ESP_LOGI(TAG, "Trying backward");
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    // }
}
