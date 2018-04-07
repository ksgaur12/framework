#pragma once

#include <modules/spi_device/spi_device.h>

#define PIXART_REG_PRODUCT_ID  0x00
#define PIXART_REG_REVISION_ID 0x01
#define PIXART_REG_MOTION      0x02
#define PIXART_REG_DELTA_X_L   0x03
#define PIXART_REG_DELTA_X_H   0x04
#define PIXART_REG_DELTA_Y_L   0x05
#define PIXART_REG_DELTA_Y_H   0x06
#define PIXART_REG_SQUAL       0x07
#define PIXART_REG_RAWDATA_SUM 0x08
#define PIXART_REG_RAWDATA_MAX 0x09
#define PIXART_REG_RAWDATA_MIN 0x0A
#define PIXART_REG_SHUTTER_LOW 0x0B
#define PIXART_REG_SHUTTER_HI  0x0C
#define PIXART_REG_CONFIG1     0x0F
#define PIXART_REG_CONFIG2     0x10
#define PIXART_REG_FRAME_CAP   0x12
#define PIXART_REG_SROM_EN     0x13
#define PIXART_REG_RUN_DS      0x14
#define PIXART_REG_REST1_RATE  0x15
#define PIXART_REG_REST1_DS    0x16
#define PIXART_REG_REST2_RATE  0x17
#define PIXART_REG_REST2_DS    0x18
#define PIXART_REG_REST3_RATE  0x19
#define PIXART_REG_OBS         0x24
#define PIXART_REG_DOUT_L      0x25
#define PIXART_REG_DOUT_H      0x26
#define PIXART_REG_RAW_GRAB    0x29
#define PIXART_REG_SROM_ID     0x2A
#define PIXART_REG_POWER_RST   0x3A
#define PIXART_REG_SHUTDOWN    0x3B
#define PIXART_REG_INV_PROD_ID 0x3F
#define PIXART_REG_INV_PROD_ID2 0x5F // for 3901
#define PIXART_REG_MOT_BURST   0x50
#define PIXART_REG_MOT_BURST2  0x16
#define PIXART_REG_SROM_BURST  0x62
#define PIXART_REG_RAW_BURST   0x64

// writing to registers needs this flag
#define PIXART_WRITE_FLAG      0x80

// timings in microseconds
#define PIXART_Tsrad           300

// correct result for SROM CRC
#define PIXART_SROM_CRC_RESULT 0xBEEF


enum pixart_type_t{
	PIXART_TYPE_3900,
	PIXART_TYPE_3901,
};


struct pixart_instance_s{
	struct spi_device_s spi_dev;
	enum pixart_type_t pixart_type;
};

struct pixart_motion_burst_s{
    uint8_t motion;
    uint8_t observation;
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal;
    uint8_t rawdata_sum;
    uint8_t max_raw;
    uint8_t min_raw;
    uint8_t shutter_upper;
    uint8_t shutter_lower;
};

union pixart_motion_busrt_u{
	struct pixart_motion_burst_s burst_struct;
	uint8_t burst_bytes[12];
};



bool pixart_init(struct pixart_instance_s* instance, uint8_t spi_idx, uint32_t select_line, enum pixart_type_t pixart_type);
void pixart_read_motion_burst(struct pixart_instance_s* instance, struct pixart_motion_burst_s* motion_burst);
