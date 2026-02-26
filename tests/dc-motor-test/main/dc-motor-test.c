/*
this program requires the bdc_motor component
in the project directory run:
idf.py add-dependency "espressif/bdc_motor^0.2.1"

source: https://components.espressif.com/components/espressif/bdc_motor/versions/0.2.1/readme

example: https://github.com/espressif/esp-idf/blob/master/examples/peripherals/mcpwm/mcpwm_bdc_speed_control/main/mcpwm_bdc_control_example_main.c
*/


#include <stdio.h>
#include "esp_log.h"
#include "bdc_motor.h"
#include "esp_timer.h"

static const char *TAG = "dc-motor-test";

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              15

typedef struct {
    bdc_motor_handle_t motor;
    // pcnt_unit_handle_t pcnt_encoder;
    // pid_ctrl_block_handle_t pid_ctrl;
    // int report_pulses;
} motor_control_context_t;

void app_main(void)
{
    static motor_control_context_t motor_ctrl_ctx = {
        // .pcnt_encoder = NULL,
        .motor = NULL,
    };

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
    motor_ctrl_ctx.motor = motor;
}
