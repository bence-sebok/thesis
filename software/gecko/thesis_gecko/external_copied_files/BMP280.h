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
#define BMP280_CHIPID_REGISTER (0xD0)
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
#define BMP280_CHIP_ID1	(0x56)
#define BMP280_CHIP_ID2	(0x57)
#define BMP280_CHIP_ID3	(0x58)
/*! @name Register addresses */
/*! @name Calibration parameter registers */
#define BMP280_DIG_T1_LSB_ADDR	 (0x88)
#define BMP280_DIG_T1_MSB_ADDR	 (0x89)
#define BMP280_DIG_T2_LSB_ADDR	 (0x8A)
#define BMP280_DIG_T2_MSB_ADDR	 (0x8B)
#define BMP280_DIG_T3_LSB_ADDR	 (0x8C)
#define BMP280_DIG_T3_MSB_ADDR	 (0x8D)
#define BMP280_DIG_P1_LSB_ADDR	 (0x8E)
#define BMP280_DIG_P1_MSB_ADDR	 (0x8F)
#define BMP280_DIG_P2_LSB_ADDR	 (0x90)
#define BMP280_DIG_P2_MSB_ADDR	 (0x91)
#define BMP280_DIG_P3_LSB_ADDR	 (0x92)
#define BMP280_DIG_P3_MSB_ADDR	 (0x93)
#define BMP280_DIG_P4_LSB_ADDR	 (0x94)
#define BMP280_DIG_P4_MSB_ADDR	 (0x95)
#define BMP280_DIG_P5_LSB_ADDR	 (0x96)
#define BMP280_DIG_P5_MSB_ADDR	 (0x97)
#define BMP280_DIG_P6_LSB_ADDR	 (0x98)
#define BMP280_DIG_P6_MSB_ADDR	 (0x99)
#define BMP280_DIG_P7_LSB_ADDR	 (0x9A)
#define BMP280_DIG_P7_MSB_ADDR	 (0x9B)
#define BMP280_DIG_P8_LSB_ADDR	 (0x9C)
#define BMP280_DIG_P8_MSB_ADDR	 (0x9D)
#define BMP280_DIG_P9_LSB_ADDR	 (0x9E)
#define BMP280_DIG_P9_MSB_ADDR	 (0x9F)
/*! @name Other registers */
#define BMP280_CHIP_ID_ADDR	(0xD0)

/*! @name Calibration parameters' relative position */
#define	BMP280_DIG_T1_LSB_POS	 (0)
#define	BMP280_DIG_T1_MSB_POS	 (1)
#define	BMP280_DIG_T2_LSB_POS	 (2)
#define	BMP280_DIG_T2_MSB_POS	 (3)
#define	BMP280_DIG_T3_LSB_POS	 (4)
#define	BMP280_DIG_T3_MSB_POS	 (5)
#define	BMP280_DIG_P1_LSB_POS	 (6)
#define	BMP280_DIG_P1_MSB_POS	 (7)
#define	BMP280_DIG_P2_LSB_POS	 (8)
#define	BMP280_DIG_P2_MSB_POS	 (9)
#define	BMP280_DIG_P3_LSB_POS	 (10)
#define	BMP280_DIG_P3_MSB_POS	 (11)
#define	BMP280_DIG_P4_LSB_POS	 (12)
#define	BMP280_DIG_P4_MSB_POS	 (13)
#define	BMP280_DIG_P5_LSB_POS	 (14)
#define	BMP280_DIG_P5_MSB_POS	 (15)
#define	BMP280_DIG_P6_LSB_POS	 (16)
#define	BMP280_DIG_P6_MSB_POS	 (17)
#define	BMP280_DIG_P7_LSB_POS	 (18)
#define	BMP280_DIG_P7_MSB_POS	 (19)
#define	BMP280_DIG_P8_LSB_POS	 (20)
#define	BMP280_DIG_P8_MSB_POS	 (21)
#define	BMP280_DIG_P9_LSB_POS	 (22)
#define	BMP280_DIG_P9_MSB_POS	 (23)
#define BMP280_CALIB_DATA_SIZE	 (24)

#define BMP280_CALIB_PARAMETERS_SIZE (24)

/*! @name Calibration parameters' structure */
struct bmp280_calib_param {
	uint16_t dig_t1;
	int16_t dig_t2;
	int16_t dig_t3;
	uint16_t dig_p1;
	int16_t dig_p2;
	int16_t dig_p3;
	int16_t dig_p4;
	int16_t dig_p5;
	int16_t dig_p6;
	int16_t dig_p7;
	int16_t dig_p8;
	int16_t dig_p9;
	int32_t t_fine;
};

I2C_TransferReturn_TypeDef BMP280_I2C_ReadRegister(uint8_t registerAddress, int8_t * bufferRead, uint16_t readSize);
I2C_TransferReturn_TypeDef BMP280_I2C_WriteRegister(uint8_t registerAddress, int8_t * bufferWrite, uint16_t sizeWrite);
void BMP280_GetCalibrationParameters(struct bmp280_calib_param * parameters);
int32_t BMP280_CompensateTemperature(uint32_t uncomp_temp, struct bmp280_calib_param *parameters);
uint32_t BMP280_CompensatePressure(uint32_t uncomp_pres, struct bmp280_calib_param *parameters);
void BMP280_SetExampleCalibrationParameters(struct bmp280_calib_param *parameters);

#endif /* BMP280_H_ */
