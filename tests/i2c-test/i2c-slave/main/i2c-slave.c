#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "hal/i2c_types.h"
#include "portmacro.h"

// error handling defines
#define TAG_I2C "I2C_SLAVE"

// Define I2C for master
#define I2C_SLAVE_SCL_IO            22
#define I2C_SLAVE_SDA_IO            21
#define I2C_SLAVE_NUM               I2C_NUM_1
#define I2C_SLAVE_TX_BUF_LEN        256
#define I2C_SLAVE_RX_BUF_LEN        256
#define I2C_SLAVE_ADDR              0x28


static esp_err_t i2c_slave_init(void)
{
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR
    };

    esp_err_t err = i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    return i2c_driver_install(
        I2C_SLAVE_NUM, 
        conf_slave.mode, 
        I2C_SLAVE_RX_BUF_LEN, 
        I2C_SLAVE_TX_BUF_LEN, 
        0
    );
}


void app_main(void)
{
    ESP_ERROR_CHECK(i2c_slave_init());
    ESP_LOGI(TAG_I2C, "I2C Slave initialized successfully.");

    uint8_t rx_data[128] = {0}; // buffer to hold received data

    while (1) {
        int rx_size = i2c_slave_read_buffer(
            I2C_SLAVE_NUM,
            rx_data, 
            sizeof(rx_data)-1,
            1000/portTICK_PERIOD_MS
        );

        if (rx_size > 0) {
            rx_data[rx_size] = '\0'; // null-terminate the string
            ESP_LOGI(TAG_I2C, "Received %d bytes: %s", rx_size, rx_data);
        } else if (rx_size < 0) {
            ESP_LOGE(TAG_I2C, "Error in i2c_slave_read_buffer data size: %d", rx_size);
        } else {
            ESP_LOGI(TAG_I2C, "No data received.");
        }
    }

}
