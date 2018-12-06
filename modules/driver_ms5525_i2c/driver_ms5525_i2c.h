#pragma once
#include "hal.h"
#include "ch.h"

#define MS5525D0_I2C_ADDR_1 0x76
#define MS5525D0_I2C_ADDR_2 0x77

#define REG_RESET               0x1E
#define REG_CONVERT_D1_OSR_256  0x40
#define REG_CONVERT_D1_OSR_512  0x42
#define REG_CONVERT_D1_OSR_1024 0x44
#define REG_CONVERT_D1_OSR_2048 0x46
#define REG_CONVERT_D1_OSR_4096 0x48
#define REG_CONVERT_D2_OSR_256  0x50
#define REG_CONVERT_D2_OSR_512  0x52
#define REG_CONVERT_D2_OSR_1024 0x54
#define REG_CONVERT_D2_OSR_2048 0x56
#define REG_CONVERT_D2_OSR_4096 0x58
#define REG_ADC_READ            0x00
#define REG_PROM_BASE           0xA0

#define REG_CONVERT_PRESSURE    REG_CONVERT_D1_OSR_1024
#define REG_CONVERT_TEMPERATURE REG_CONVERT_D2_OSR_1024

typedef struct{
    I2CDriver* bus;
    I2CConfig* bus_config;
    uint8_t address;
    uint16_t prom[8];
    int32_t D1;
    int32_t D2;
    float press;
    float temp;
}MS5525_dev_t;

typedef MS5525_dev_t* MS5525_dev;

uint8_t read_prom(MS5525_dev dev);

void ms5525_i2c_init(MS5525_dev dev);

void ms5525_i2c_press_data(MS5525_dev dev);

void ms5525_i2c_temp_data(MS5525_dev dev);

void ms5525_calculate_airspeed(MS5525_dev dev);

void restart_i2c(MS5525_dev dev);

void reset_ms5525(MS5525_dev dev);

void ms5525_read_prom(MS5525_dev dev);

void broadcast_MS5525(MS5525_dev dev);

