#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "portmacro.h"

// error handling defines
#define TAG_I2C "I2C_MASTER"

// Define I2C for master
#define I2C_MASTER_SCL_IO           22              /* GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21              /* GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0       /* I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000          /* I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0               /* I2C master do not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0               /* I2C master do not need buffer */

// slave
#define I2C_SLAVE_ADDR              0x28            /* need to figure this out */


static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    return i2c_driver_install(
        I2C_MASTER_NUM, 
        conf.mode, 
        I2C_MASTER_RX_BUF_DISABLE, 
        I2C_MASTER_TX_BUF_DISABLE, 
        0
    );
}

static esp_err_t i2c_master_write_slave(
    i2c_port_t i2c_num,
    uint8_t *data_wr,
    size_t size
)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(
        cmd,
        (I2C_SLAVE_ADDR << 1) | I2C_MASTER_WRITE,
        true
    ); // send address, wait for ack --> means slave has been found
    i2c_master_write(
        cmd,
        data_wr,
        size,
        true
    ); // send data, wait for ack
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(
        i2c_num, 
        cmd, 
        1000/portTICK_PERIOD_MS
    );
    i2c_cmd_link_delete(cmd);
    return ret;
}

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG_I2C, "I2C Master initialized successfully.");

    uint8_t data_to_send[] = "Hello from Master!";

    while (1) {
        esp_err_t ret = i2c_master_write_slave(
            I2C_MASTER_NUM,
            data_to_send,
            sizeof(data_to_send)
        );
        if (ret == ESP_OK) {
            ESP_LOGI(TAG_I2C, "Sent: %s", data_to_send);
        } else {
            ESP_LOGE(
                TAG_I2C, 
                "Failed to write to slave (0x%x). Error: %s",
                I2C_SLAVE_ADDR,
                esp_err_to_name(ret)
            );
        }
        vTaskDelay(2000/portTICK_PERIOD_MS); // wait 2 secs
    }



}
