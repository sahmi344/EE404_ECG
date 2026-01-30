#include "ads1115.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define ADS1115_OS_SINGLE      0x8000
#define ADS1115_MODE_SINGLE   0x0100
#define ADS1115_DR_128SPS     0x0080
#define ADS1115_COMP_DISABLE  0x0003

esp_err_t ads1115_init(i2c_port_t port)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(port, &conf);
    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}

int16_t ads1115_read_single(i2c_port_t port, uint8_t channel)
{
    uint16_t mux;

    switch (channel) {
        case 0: mux = 0x4000; break;
        case 1: mux = 0x5000; break;
        case 2: mux = 0x6000; break;
        case 3: mux = 0x7000; break;
        default: return 0;
    }

    uint16_t config =
        ADS1115_OS_SINGLE |
        mux |
        ADS1115_GAIN_ONE |
        ADS1115_MODE_SINGLE |
        ADS1115_DR_128SPS |
        ADS1115_COMP_DISABLE;

    uint8_t tx[3] = {
        ADS1115_REG_CONFIG,
        (uint8_t)(config >> 8),
        (uint8_t)(config & 0xFF)
    };

    i2c_master_write_to_device(
        port, ADS1115_ADDR,
        tx, 3, pdMS_TO_TICKS(100));

    vTaskDelay(pdMS_TO_TICKS(9));

    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t rx[2];

    i2c_master_write_read_device(
        port, ADS1115_ADDR,
        &reg, 1, rx, 2,
        pdMS_TO_TICKS(100));

    return (int16_t)((rx[0] << 8) | rx[1]);
}