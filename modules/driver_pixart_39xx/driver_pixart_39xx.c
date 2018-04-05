#include "driver_pixart_39xx.h"
#include <common/bswap.h>

#define MODULE_UAVCAN_DEBUG_ENABLED 1

#ifdef MODULE_UAVCAN_DEBUG_ENABLED
#include <modules/uavcan_debug/uavcan_debug.h>
#define PIXART_DEBUG(...) uavcan_send_debug_msg(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "PIXART", __VA_ARGS__)
#else
#define PIXART_DEBUG(...) {}
#endif

static void pixart_reg_write(struct pixart_instance_s* instance, uint8_t reg, uint8_t value);
static uint8_t pixart_reg_read(struct pixart_instance_s* instance, uint8_t reg);

static bool check_pixart_device_id(struct pixart_instance_s* instance);

bool pixart_init(struct pixart_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum pixart_type_t pixart_type){

	PIXART_DEBUG("pixart_init");

    spi_device_init(&instance->spi_dev, spi_idx, select_line, 10000, 8, SPI_DEVICE_FLAG_CPHA|SPI_DEVICE_FLAG_CPOL);

    spi_device_set_max_speed_hz(&instance->spi_dev, 10000000);

	pixart_reg_write(instance, PIXART_REG_POWER_RST, 0x5A);
	chThdSleep(MS2ST(50));

	if(!check_pixart_device_id(instance)){
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

static bool check_pixart_device_id(struct pixart_instance_s* instance){
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
