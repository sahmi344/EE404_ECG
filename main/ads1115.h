#ifndef ADS1115_H
#define ADS1115_H

#include "driver/i2c.h"
#include "esp_err.h"

#define ADS1115_ADDR               0x48
#define ADS1115_REG_CONVERSION     0x00
#define ADS1115_REG_CONFIG         0x01

// GAIN_ONE → ±4.096V → 0.125 mV per bit
#define ADS1115_GAIN_ONE           0x0200

esp_err_t ads1115_init(i2c_port_t port);
int16_t ads1115_read_single(i2c_port_t port, uint8_t channel);

#endif