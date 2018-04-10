#include "driver_pixart_39xx.h"
#include <common/bswap.h>
#include <modules/uavcan/uavcan.h>
#include <string.h>


#define MODULE_UAVCAN_DEBUG_ENABLED 1

#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#define PIXART_DEBUG(...) uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "PIXART", __VA_ARGS__)
#else
#define PIXART_DEBUG(...) {}
#endif


static void pixart_reg_write(struct pixart_instance_s* instance, uint8_t reg, uint8_t value);
static uint8_t pixart_reg_read(struct pixart_instance_s* instance, uint8_t reg);

static bool pixart_check_device_id(struct pixart_instance_s* instance);

bool pixart_init(struct pixart_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum pixart_type_t pixart_type){

	(void)pixart_type;

	PIXART_DEBUG("pixart_init");

    spi_device_init(&instance->spi_dev, spi_idx, select_line, 10000, 8, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);

    spi_device_set_max_speed_hz(&instance->spi_dev, 10000000);

	pixart_reg_write(instance, PIXART_REG_POWER_RST, 0x5A);
	chThdSleep(MS2ST(50));

	if(!pixart_check_device_id(instance)){
		PIXART_DEBUG("cannot detect pixart 39xx");
		return false;
	}

	PIXART_DEBUG("pixart 3901 detected");

	return true;
}

static void pixart_reg_write(struct pixart_instance_s* instance, uint8_t reg, uint8_t value){
	spi_device_begin(&instance->spi_dev);
    reg |= PIXART_WRITE_FLAG;
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    chThdSleep(US2ST(PIXART_Tsrad));
    spi_device_send(&instance->spi_dev, sizeof(value), &value);
    spi_device_end(&instance->spi_dev);
    chThdSleep(US2ST(120));
}


static uint8_t pixart_reg_read(struct pixart_instance_s* instance, uint8_t reg){
    uint8_t v = 0;
    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    chThdSleep(US2ST(20));
    spi_device_receive(&instance->spi_dev, sizeof(v), &v);
    spi_device_end(&instance->spi_dev);
    chThdSleep(US2ST(200));
    return v;
}

static bool pixart_check_device_id(struct pixart_instance_s* instance){
	uint8_t id1 = pixart_reg_read(instance, PIXART_REG_PRODUCT_ID);
	uint8_t id2 = pixart_reg_read(instance, PIXART_REG_INV_PROD_ID2);
	PIXART_DEBUG("id1 %x", id1);
	PIXART_DEBUG("~id1 %x", (uint8_t)(~id1));
	PIXART_DEBUG("id2 %x", id2);
	if(id1 == 0x49 && id2 == (uint8_t)(~id1)){
		return true;
	}
	return false;
}

void pixart_read_motion_burst(struct pixart_instance_s* instance, struct pixart_motion_burst_s* motion_burst){

	uint8_t b[12];

    uint8_t reg = PIXART_REG_MOT_BURST2;

    spi_device_begin(&instance->spi_dev);
    spi_device_send(&instance->spi_dev, sizeof(reg), &reg);
    chThdSleep(US2ST(150));
    spi_device_receive(&instance->spi_dev, sizeof(b), &b);
    spi_device_end(&instance->spi_dev);
    memcpy(motion_burst, b, sizeof(b));
}

void pixart_flow_msg_broadcast(int16_t _delta_x, int16_t _delta_y, uint8_t _quality){

	struct hex_optic_flow_OpticFlow_s flow_msg;

	flow_msg.raw_flow_x = _delta_x;
	flow_msg.raw_flow_y = _delta_y;
	flow_msg.qaulity = _quality;

    uavcan_broadcast(0, &hex_optic_flow_OpticFlow_descriptor, CANARD_TRANSFER_PRIORITY_LOWEST, &flow_msg);

}
