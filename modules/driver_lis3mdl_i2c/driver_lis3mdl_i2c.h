#include "hal.h"
#include "ch.h"


#define LIS3MDL_I2C_ADDR 0x1c

#define ADDR_CTRL_REG1      0x20
#define ADDR_CTRL_REG2      0x21
#define ADDR_CTRL_REG3      0x22
#define ADDR_CTRL_REG4      0x23
#define ADDR_CTRL_REG5      0x24

#define ADDR_STATUS_REG     0x27
#define ADDR_OUT_X_L        0x28
#define ADDR_OUT_X_H        0x29
#define ADDR_OUT_Y_L        0x2a
#define ADDR_OUT_Y_H        0x2b
#define ADDR_OUT_Z_L        0x2c
#define ADDR_OUT_Z_H        0x2d
#define ADDR_OUT_T_L        0x2e
#define ADDR_OUT_T_H        0x2f

#define MODE_REG_CONTINOUS_MODE     (0 << 0)
#define MODE_REG_SINGLE_MODE        (1 << 0)

#define ADDR_WHO_AM_I       0x0f
#define ID_WHO_AM_I         0x3d

typedef struct{
    I2CDriver* bus;
    I2CConfig* bus_config;
    uint8_t address;
    float mag[3];
}LIS3MDL_dev_t;

enum Mag_status{
    NONE,
    FOUND,
    INITIALIZED,
    DATA_READY,
    DATA_READ
};


typedef LIS3MDL_dev_t* LIS3MDL_dev;

//uint8_t read_prom(MS5525_dev dev);
//
//void ms5525_i2c_init(MS5525_dev dev);
//
//void ms5525_i2c_press_data(MS5525_dev dev);
//
//void ms5525_i2c_temp_data(MS5525_dev dev);
//
//void ms5525_calculate_airspeed(MS5525_dev dev);
//
//void restart_i2c(MS5525_dev dev);
//
//void reset_ms5525(MS5525_dev dev);
//
//void ms5525_read_prom(MS5525_dev dev);
//

void lis3mdl_init(LIS3MDL_dev dev);

void write_register(LIS3MDL_dev dev, uint8_t reg, uint8_t val, char str[]);

void blink_leds();

void get_data(LIS3MDL_dev dev);

void broadcast_MMC3416(LIS3MDL_dev dev);

