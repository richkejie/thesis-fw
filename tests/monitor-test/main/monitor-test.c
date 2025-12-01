#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    while(1) {
        printf("Monitor test app is running...\n");
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
