
#include "driver_ms5525_i2c.h"
#include <modules/uavcan_debug/uavcan_debug.h>
#include <uavcan.equipment.air_data.RawAirData.h>

//Calculations and code from https://raw.githubusercontent.com/ArduPilot/ardupilot/master/libraries/AP_Airspeed/AP_Airspeed_MS5525.cpp

uint16_t crc4_prom(MS5525_dev dev){

    uint16_t n_rem = 0;
    uint8_t n_bit;

    for (uint8_t cnt = 0; cnt < sizeof(dev->prom); cnt++) {
        /* uneven bytes */
        if (cnt & 1) {
            n_rem ^= (uint8_t)((dev->prom[cnt >> 1]) & 0x00FF);
        } else {
            n_rem ^= (uint8_t)(dev->prom[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            } else {
                n_rem = (n_rem << 1);
            }
        }
    }

    return (n_rem >> 12) & 0xF;
}

void ms5525_i2c_init(MS5525_dev dev){

	i2cStart(dev->bus, dev->bus_config);

	chThdSleepMilliseconds(500);

	reset_ms5525(dev);

	chThdSleepMilliseconds(500);

	ms5525_read_prom(dev);

}

void ms5525_read_prom(MS5525_dev dev){

	msg_t msg = MSG_TIMEOUT;

	uint8_t data[16];

	uint8_t cmd[2];

	cmd[0] = REG_RESET;

	chThdSleepMilliseconds(500);

	for(uint8_t i = 0; i < 8; i++){

		i2cAcquireBus(dev->bus);

		cmd[0] = REG_PROM_BASE+2*i;

		msg = i2cMasterTransmitTimeout(dev->bus, dev->address, cmd, 1,
								   data, 2, TIME_INFINITE);

		if (msg != MSG_OK){
			uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"", "%u ms5525_i2c_init msg %d", i, msg);
			restart_i2c(dev);
		}else{
			dev->prom[i] = (uint16_t)((data[0] << 8) | (data[1]));
			uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"", "%u ms5525_i2c prom success %x, %x, %u", i, data[0], data[1], dev->prom[i]);
		}

		chThdSleepMilliseconds(20);
		i2cReleaseBus(dev->bus);
	}

    const uint16_t crc_read = dev->prom[7] & 0xf;

    dev->prom[7] &= 0xff00;

    uint16_t crc_calc = crc4_prom(dev);
    if (crc_read != crc_calc) {
    	uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"","MS5525: CRC mismatch 0x%04x 0x%04x\n", crc_read, crc_calc);
    }else{
    	uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"","MS5525: CRC success 0x%04x 0x%04x\n", crc_read, crc_calc);
    }

}


void ms5525_i2c_press_data(MS5525_dev dev){

	msg_t msg = MSG_TIMEOUT;

	uint8_t cmd[2];

	uint8_t val[3];

	chThdSleepMilliseconds(5);

	cmd[0] = REG_CONVERT_PRESSURE;

	i2cAcquireBus(dev->bus);

	msg = i2cMasterTransmitTimeout(dev->bus, dev->address, cmd, 1,
							   val, 0, MS2ST(10));
	if (msg != MSG_OK){
		uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"",
				"ms5525_i2c_read_press_data fail msg %d ", msg);
		restart_i2c(dev);
	}

	i2cReleaseBus(dev->bus);

	chThdSleepMilliseconds(5);

	cmd[0] = REG_ADC_READ;

	i2cAcquireBus(dev->bus);

	msg = i2cMasterTransmitTimeout(dev->bus, dev->address, cmd, 1,
							   val, 3, MS2ST(10));
	if (msg != MSG_OK){
		uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"",
				"ms5525_i2c_read_press_data fail  msg %d", msg);
		restart_i2c(dev);
	}else{
		int32_t adc_val =  (val[0] << 16) | (val[1] << 8) | val[2];
		dev->D1 = adc_val;
	}

	i2cReleaseBus(dev->bus);
}


void ms5525_i2c_temp_data(MS5525_dev dev){

	msg_t msg = MSG_TIMEOUT;

	uint8_t cmd[2];

	uint8_t val[3];

	chThdSleepMilliseconds(5);

	cmd[0] = REG_CONVERT_TEMPERATURE;

	i2cAcquireBus(dev->bus);

	msg = i2cMasterTransmitTimeout(dev->bus, dev->address, cmd, 1,
							   val, 0, MS2ST(10));
	if (msg != MSG_OK){
		uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"",
				"ms5525_i2c_read_temp_data fail msg %d ", msg);
		restart_i2c(dev);
	}

	i2cReleaseBus(dev->bus);

	chThdSleepMilliseconds(5);

	cmd[0] = REG_ADC_READ;

	i2cAcquireBus(dev->bus);

	msg = i2cMasterTransmitTimeout(dev->bus, dev->address, cmd, 1,
							   val, 3, MS2ST(10));
	if (msg != MSG_OK){
		uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"",
				"ms5525_i2c_read_temp_data fail  msg %d", msg);
		restart_i2c(dev);
	}else{
		int32_t adc_val =  (val[0] << 16) | (val[1] << 8) | val[2];
		dev->D2 = adc_val;
	}

	i2cReleaseBus(dev->bus);
}

void ms5525_calculate_airspeed(MS5525_dev dev){

    ms5525_i2c_press_data(dev);

    ms5525_i2c_temp_data(dev);

    const uint8_t Q1 = 15;
    const uint8_t Q2 = 17;
    const uint8_t Q3 = 7;
    const uint8_t Q4 = 5;
    const uint8_t Q5 = 7;
    const uint8_t Q6 = 21;

    int64_t dT = dev->D2 - (dev->prom[5]) * (1UL<<Q5);
    int64_t TEMP = 2000 + (dT*(dev->prom[6]))/(1UL<<Q6);
    int64_t OFF =  (dev->prom[2])*(1UL<<Q2) + ((dev->prom[4])*dT)/(1UL<<Q4);
    int64_t SENS = (dev->prom[1])*(1UL<<Q1) + ((dev->prom[3])*dT)/(1UL<<Q3);
    int64_t P = (dev->D1*SENS/(1UL<<21)-OFF)/(1UL<<15);
    const float PSI_to_Pa = 6894.757f;
    float P_Pa = PSI_to_Pa * 1.0e-4 * P;
    float Temp_C = TEMP * 0.01;

    dev->press = P_Pa;
    dev->temp = Temp_C;
}


void reset_ms5525(MS5525_dev dev){

	msg_t msg = MSG_TIMEOUT;

	uint8_t data[16];

	uint8_t cmd[2];
	cmd[0] = REG_RESET;
	cmd[1] = REG_PROM_BASE;

	uint8_t reset_retires = 5;

	while(msg != MSG_OK && reset_retires > 0){

		i2cAcquireBus(dev->bus);

		msg = i2cMasterTransmitTimeout(dev->bus, dev->address, cmd, 1,
								   data, 0, MS2ST(10));
		if (msg != MSG_OK){
			uavcan_send_debug_msg(LOG_LEVEL_DEBUG,"",
					"ms5525_i2c_reset msg %d retries %u", msg, reset_retires);

			i2cReleaseBus(dev->bus);

			i2cStop(dev->bus);

			chThdSleepMilliseconds(50);

			i2cStart(dev->bus, dev->bus_config);

			i2cAcquireBus(dev->bus);

			chThdSleepMilliseconds(50);

		}

		i2cReleaseBus(dev->bus);

		chThdSleepMilliseconds(500);

		reset_retires--;
	}
}

void restart_i2c(MS5525_dev dev){

	i2cReleaseBus(dev->bus);

	i2cStop(dev->bus);

	chThdSleepMilliseconds(50);

	i2cStart(dev->bus, dev->bus_config);

	i2cAcquireBus(dev->bus);

	chThdSleepMilliseconds(50);

	reset_ms5525(dev);

}

void broadcast_MS5525(MS5525_dev dev){

	static struct uavcan_equipment_air_data_RawAirData_s msg;

	msg.differential_pressure = dev->press;
	msg.differential_pressure_sensor_temperature = dev->press + 273.15; //in Kelvin

	uavcan_broadcast(0, &uavcan_equipment_air_data_RawAirData_descriptor, CANARD_TRANSFER_PRIORITY_MEDIUM, &msg);


}
