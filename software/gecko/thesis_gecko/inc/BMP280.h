/*
 * BMP280.h
 *
 *  Created on: Nov 5, 2018
 *      Author: sebokbence
 */

#include "em_gpio.h"
#include "em_i2c.h"

#ifndef BMP280_H_
#define BMP280_H_

/*! @name Sensor configuration structure */
struct bmp280_config {
	uint8_t os_temp;
	uint8_t os_pres;
	uint8_t odr;
	uint8_t filter;
	uint8_t spi3w_en;
};

// BMP280 defines
// Connecting SDO to GND results in slave address 1110110 (0x76)
#define I2C_ADDRESS_BMP280 (0x76)
#define BMP280_CHIPID (0x58)
/*! @name Return codes */
/*! @name Success code*/
#define BMP280_OK			(0)
/*! @name Error codes */
#define BMP280_E_NULL_PTR		(-1)
#define BMP280_E_DEV_NOT_FOUND		(-2)
#define BMP280_E_INVALID_LEN		(-3)
#define BMP280_E_COMM_FAIL		(-4)
#define BMP280_E_INVALID_MODE		(-5)
/*! @name Chip IDs */
#define BMP280_CHIP_ID1	UINT8_C(0x56)
#define BMP280_CHIP_ID2	UINT8_C(0x57)
#define BMP280_CHIP_ID3	UINT8_C(0x58)
/*! @name Other registers */
#define BMP280_CHIP_ID_ADDR	(0xD0)

// I2C address
#define I2C_ADDRESS_7BIT_SI7051 (0x40)
#define I2C_ADDRESS_8BIT_SI7051 (I2C_ADDRESS_7BIT_SI7051 << 1)
#define SI7051_I2C (I2C0)
enum si7051_command {
	SI7051_COMMAND_READ_FIRMWARE_VERSION = 0,
	SI7051_COMMAND_MEASUREMENT_NOHOLD = 1,
	SI7051_COMMAND_MEASUREMENT_HOLD = 2,
	SI7051_COMMAND_READ_USERREGISTER1 = 3,
	SI7051_COMMAND_RESET = 4
};

#endif /* BMP280_H_ */
