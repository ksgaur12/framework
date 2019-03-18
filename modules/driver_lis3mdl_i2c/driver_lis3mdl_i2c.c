
#include "driver_lis3mdl_i2c.h"
#include <modules/uavcan_debug/uavcan_debug.h>
#include <uavcan.equipment.ahrs.MagneticFieldStrength.h>

//Calculations and code from https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Compass/AP_Compass_LIS3MDL.cpp

enum Mag_status LIS3MDL_status = NONE;

void write_register(LIS3MDL_dev dev, uint8_t reg, uint8_t val, char str[]){

    msg_t msg = MSG_TIMEOUT;
    uint8_t buf[2] = { reg, val };
    uint8_t data;
    msg = i2cMasterTransmitTimeout(dev->bus, dev->address, buf, sizeof(buf), &data, 0, MS2ST(50));

    if(msg == MSG_OK){
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"", "%s", str);
    }
    chThdSleepMilliseconds(10);
}

void lis3mdl_init(LIS3MDL_dev dev){

    msg_t msg = MSG_TIMEOUT;

    uint8_t cmd = ADDR_WHO_AM_I;
    uint8_t data;

    i2cStart(dev->bus, dev->bus_config);
    chThdSleepMilliseconds(500);

    uint8_t retries = 10;

    for(uint8_t i = 0; i < retries; i++){
        msg = i2cMasterTransmitTimeout(dev->bus, dev->address, &cmd, 1, &data, 1, MS2ST(50)); //check for mag device address

        if(data == ID_WHO_AM_I  ){ //mag is found
            LIS3MDL_status = FOUND;
            uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"", "Mag LIS found", data);
            break;
        }
    }

    if(LIS3MDL_status != FOUND){
        uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"", "Mag LIS not found");
        return;
    }

    write_register(dev, ADDR_CTRL_REG1, 0xFC, "rate set to 80Hz"); // 80Hz, UHP
    write_register(dev, ADDR_CTRL_REG2, 0, "range set to 4Ga"); // 4Ga range
    write_register(dev, ADDR_CTRL_REG3, 0, "mode set to continuous"); // continuous
    write_register(dev, ADDR_CTRL_REG4, 0x0C, "ultra high pref in z-axis"); // z-axis ultra high perf
    write_register(dev, ADDR_CTRL_REG5, 0x40, "block data update"); // block-data-update
    LIS3MDL_status = INITIALIZED;
}

void get_data(LIS3MDL_dev dev){
    if(LIS3MDL_status == INITIALIZED){
        uint8_t cmd = ADDR_STATUS_REG;

        // check data ready
        uint8_t status;

        i2cMasterTransmitTimeout(dev->bus, dev->address, &cmd, 1, &status, 1, MS2ST(10)); //check for data ready

        if ((status & 0x08)) {
            LIS3MDL_status = DATA_READY;
        }
    }
    else if(LIS3MDL_status == DATA_READY){

        const float range_scale = 1000.0 / 6842.0;

        uint8_t cmd = ADDR_OUT_X_L;

        struct PACKED {
            int16_t magx;
            int16_t magy;
            int16_t magz;
        } data;

        i2cMasterTransmitTimeout(dev->bus, dev->address, &cmd, 1, (uint8_t *)&data, sizeof(data), MS2ST(50)); //read data

        dev->mag[0] = data.magx * range_scale;
        dev->mag[1] = data.magy * range_scale;
        dev->mag[2] = data.magz * range_scale;

        broadcast_MMC3416(dev);
        LIS3MDL_status = INITIALIZED;
    }
}

void blink_leds(){
    palTogglePad(GPIOA, 0);
    palTogglePad(GPIOA, 1);
}

void broadcast_MMC3416(LIS3MDL_dev dev){

	static struct uavcan_equipment_ahrs_MagneticFieldStrength_s msg;

	msg.magnetic_field_ga[0] = dev->mag[0];
    msg.magnetic_field_ga[1] = dev->mag[1];
    msg.magnetic_field_ga[2] = dev->mag[2];

	uavcan_broadcast(0, &uavcan_equipment_ahrs_MagneticFieldStrength_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg);
}
