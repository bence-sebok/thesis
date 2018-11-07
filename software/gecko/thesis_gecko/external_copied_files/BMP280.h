/*
 * KX126.h
 *
 *  Created on: Nov 5, 2018
 *      Author: sebokbence
 */

#include "em_gpio.h"
#include "em_i2c.h"

#ifndef KX126_H_
#define KX126_H_

// BMP280 defines
// Connecting SDO to GND results in slave address 1110110 (0x76)
#define I2C_ADDRESS_BMP280 (0x76)
#define BMP280_REGISTER_CHIPID (0xD0)
#define BMP280_REGISTER_TEMP_XLSB (0xFC)
#define BMP280_REGISTER_TEMP_LSB (0xFB)
#define BMP280_REGISTER_TEMP_MSB (0xFA)
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

/*
 * STRUCTURES
 */
/**
  * A structure to represent X, Y, Z accelerations with lower and higher bytes.
  */
typedef struct {
	/*@{*/
	int8_t xoutl; /**< the lower byte of x coordinate */
	int8_t xouth; /**< the higher byte of x coordinate */
	int8_t youtl; /**< the lower byte of y coordinate */
	int8_t youth; /**< the higher byte of y coordinate */
	int8_t zoutl; /**< the lower byte of z coordinate */
	int8_t zouth; /**< the higher byte of z coordinate */
	/*@}*/
} accel_data_lh_t;

/**
  * A structure to represent X, Y, Z accelerations with two-byte words.
  */
typedef struct {
	/*@{*/
	int16_t x; /**< the word of x coordinate */
	int16_t y; /**< the word of y coordinate */
	int16_t z; /**< the word of z coordinate */
	/*@}*/
} accel_data_w_t;

/*
 * FUNCTION DEFINITIONS
 */
void init_I2C(void);
I2C_TransferReturn_TypeDef KX126_I2C_ReadRegister(uint8_t registerAddress, int8_t * bufferRead, uint16_t readSize);
uint8_t KX126_I2C_WriteRegister(uint8_t registerAddress, uint8_t data);
accel_data_w_t KX126_ReadXYZ();
void KX126_EnableSensor(void);

/*
 * DEFINING CONTANTS
 */
// Select I2C bus I2C0 or I2C1
#define KX126_I2C (I2C0)
// I2C0 pins: I2C0_SDA is PA5, I2C0_SCL is PA4
#define I2C0_SDA_PORT (gpioPortA)
#define I2C0_SDA_PIN (5U)
#define I2C0_SCL_PORT (gpioPortA)
#define I2C0_SCL_PIN (4U)
// I2C address
#define I2C_ADDRESS_7BIT_KX126 (0x1E)
#define I2C_ADDRESS_8BIT_KX126 (0x1E << 1)
// KX126 registers
#define KX126_REGISTER_MANID (0x00)
#define KX126_REGISTER_XOUTL (0x08)
#define KX126_REGISTER_XOUTH (0x09)
#define KX126_REGISTER_YOUTL (0x0A)
#define KX126_REGISTER_YOUTH (0x0B)
#define KX126_REGISTER_ZOUTL (0x0C)
#define KX126_REGISTER_ZOUTH (0x0D)
#define KX126_REGISTER_WHOAMI (0x11)
#define KX126_REGISTER_ODCNTL (0x1F)
#define KX126_REGISTER_CNTL1 (0x1A)
#define KX126_REGISTER_CNTL2 (0x1B)
#define KX126_REGISTER_CNTL3 (0x1C)
#define KX126_REGISTER_CNTL4 (0x1D)
#define KX126_REGISTER_CNTL5 (0x1E)
// Register values to read or write
#define KX126_CNTL1_PC1_SET (1 << 7)

I2C_TransferReturn_TypeDef BMP280_I2C_ReadRegister(uint8_t registerAddress, int8_t * bufferRead, uint16_t readSize);

#endif /* KX126_H_ */
