/*
 * KX126.c
 *
 *  Created on: Nov 5, 2018
 *      Author: sebokbence
 */

#include "hardware_config.h"
#include "BMP280.h"
#include <string.h>
#include "em_i2c.h"

/**
 * @brief
 * 	init_I2C function initializes the I2C bus with proper hardware configuration.
 * @retval
 * 	none
 */
void init_I2C(void)
{
	// Using default settings
	  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	  // Use ~400khz SCK
	  i2cInit.freq = 400000;
	  // GPIO pins
	  GPIO_PinModeSet(I2C0_SDA_PORT, I2C0_SDA_PIN, gpioModeWiredAndPullUpFilter, 1);
	  GPIO_PinModeSet(I2C0_SCL_PORT, I2C0_SCL_PIN, gpioModeWiredAndPullUpFilter, 1);
	  // Enable pins at location 15 as specified in datasheet
	  I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | (1 << _I2C_ROUTE_LOCATION_SHIFT);
	  // Initializing the I2C
	  I2C_Init(I2C0, &i2cInit);
	  // Set slave address and ctrl
	  I2C0->SADDR = I2C_ADDRESS_BMP280;
	  I2C0->CTRL |= I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSN;
}

/**
 * @brief
 *  Reads data from KX126 sensor. Reads readSize bytes data from register pointed by registerAddress to bufferRead array.
 * @param[in] registerAddress
 *   The register to read from.
 * @param[in] bufferRead
 *   Buffer to read.
 * @param[out] readSize
 *   Number of bytes to read.
 * @return
 *   Status of I2C transfer.
*/
I2C_TransferReturn_TypeDef BMP280_I2C_ReadRegister(uint8_t registerAddress, int8_t * bufferRead, uint16_t readSize)
{
	// Local variables for I2C transfer
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_write_data[1] = {0};
	uint8_t i2c_write_size = 1;

	// Set seq values
	seq.addr  = I2C_ADDRESS_BMP280 << 1;
	seq.flags = I2C_FLAG_WRITE_READ;
	/* Select register address to issue */
	i2c_write_data[0] = registerAddress;
	seq.buf[0].data   = i2c_write_data;
	seq.buf[0].len    = i2c_write_size;
	/* Select location/length of data to be read */
	seq.buf[1].data = (uint8_t *)bufferRead;
	seq.buf[1].len  = readSize;

	ret = I2C_TransferInit(I2C0, &seq);
	// Sending data
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	return i2cTransferDone;
}

I2C_TransferReturn_TypeDef BMP280_I2C_WriteRegister(uint8_t registerAddress, int8_t * bufferWrite, uint16_t sizeWrite)
{
	// Local variables for I2C transfer
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t i2c_write_data[1] = {0};
	uint8_t i2c_write_size = 1;

	// Set seq values
	seq.addr  = I2C_ADDRESS_BMP280 << 1;
	seq.flags = I2C_FLAG_WRITE_WRITE;
	/* Select register address to issue */
	i2c_write_data[0] = registerAddress;
	seq.buf[0].data   = i2c_write_data;
	seq.buf[0].len    = i2c_write_size;
	/* Select location/length of data to be read */
	seq.buf[1].data = (uint8_t *)bufferWrite;
	seq.buf[1].len  = sizeWrite;

	ret = I2C_TransferInit(I2C0, &seq);
	// Sending data
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(I2C0);
	}

	return i2cTransferDone;
}

/*!
 * @brief This API is used to read the calibration parameters used
 * for calculating the compensated data.
 */
void BMP280_GetCalibrationParameters(struct bmp280_calib_param * parameters)
{
	uint8_t temp[BMP280_CALIB_PARAMETERS_SIZE] = {0};

	BMP280_I2C_ReadRegister(BMP280_DIG_T1_LSB_ADDR, (int8_t *)temp, BMP280_CALIB_PARAMETERS_SIZE);
	parameters->dig_t1 = (uint16_t) (((uint16_t) temp[BMP280_DIG_T1_MSB_POS] << 8)
	| ((uint16_t) temp[BMP280_DIG_T1_LSB_POS]));
	parameters->dig_t2 = (int16_t) (((int16_t) temp[BMP280_DIG_T2_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_T2_LSB_POS]));
	parameters->dig_t3 = (int16_t) (((int16_t) temp[BMP280_DIG_T3_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_T3_LSB_POS]));
	parameters->dig_p1 = (uint16_t) (((uint16_t) temp[BMP280_DIG_P1_MSB_POS] << 8)
	| ((uint16_t) temp[BMP280_DIG_P1_LSB_POS]));
	parameters->dig_p2 = (int16_t) (((int16_t) temp[BMP280_DIG_P2_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P2_LSB_POS]));
	parameters->dig_p3 = (int16_t) (((int16_t) temp[BMP280_DIG_P3_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P3_LSB_POS]));
	parameters->dig_p4 = (int16_t) (((int16_t) temp[BMP280_DIG_P4_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P4_LSB_POS]));
	parameters->dig_p5 = (int16_t) (((int16_t) temp[BMP280_DIG_P5_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P5_LSB_POS]));
	parameters->dig_p6 = (int16_t) (((int16_t) temp[BMP280_DIG_P6_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P6_LSB_POS]));
	parameters->dig_p7 = (int16_t) (((int16_t) temp[BMP280_DIG_P7_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P7_LSB_POS]));
	parameters->dig_p8 = (int16_t) (((int16_t) temp[BMP280_DIG_P8_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P8_LSB_POS]));
	parameters->dig_p9 = (int16_t) (((int16_t) temp[BMP280_DIG_P9_MSB_POS] << 8)
	| ((int16_t) temp[BMP280_DIG_P9_LSB_POS]));
}


/*!
 * @brief This API is used to get the compensated temperature from
 * uncompensated temperature. This API uses 32 bit integers.
 */
int32_t BMP280_CompensateTemperature(uint32_t uncomp_temp, struct bmp280_calib_param *parameters)
{
	double var1;
	double var2;
	int32_t temperature = 0;
	//var1 = ((((uncomp_temp >> 3) - ((double) parameters->dig_t1 << 1))) * ((double) parameters->dig_t2)) >> 11;
	var1 = (((double)uncomp_temp)/16384.0 - ((double)parameters->dig_t1)/1024.0) * ((double)parameters->dig_t2);
	var2 = ((((double)uncomp_temp)/131072.0 - ((double)parameters->dig_t1)/8192.0) * (((double)uncomp_temp)/131072.0 - ((double)parameters->dig_t1)/8192.0)) * ((double)parameters->dig_t3);
	parameters->t_fine = var1 + var2;
	temperature = (parameters->t_fine * 5 + 128) >> 8;
	return temperature;
}

/*!
 * @brief This API is used to get the compensated pressure from
 * uncompensated pressure. This API uses 32 bit integers.
 */
uint32_t BMP280_CompensatePressure(uint32_t uncomp_pres, struct bmp280_calib_param *parameters)
{
	int32_t var1;
	int32_t var2;
	uint32_t pressure = 0;
	var1 = (((int32_t) parameters->t_fine) >> 1) - (int32_t) 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) parameters->dig_p6);
	var2 = var2 + ((var1 * ((int32_t) parameters->dig_p5)) << 1);
	var2 = (var2 >> 2) + (((int32_t) parameters->dig_p4) << 16);
	var1 = (((parameters->dig_p3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3)
	+ ((((int32_t) parameters->dig_p2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t) parameters->dig_p1)) >> 15);
	pressure = (((uint32_t) (((int32_t) 1048576) - uncomp_pres) - (var2 >> 12))) * 3125;

	/* Avoid exception caused by division with zero */
	if (var1 != 0) {
		/* Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1 */
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((uint32_t) var1);
		else
			pressure = (pressure / (uint32_t) var1) * 2;

		var1 = (((int32_t) parameters->dig_p9)
		* ((int32_t) (((pressure >> 3) * (pressure >> 3)) >> 13))) >> 12;
		var2 = (((int32_t) (pressure >> 2)) * ((int32_t) parameters->dig_p8)) >> 13;
		pressure = (uint32_t) ((int32_t) pressure + ((var1 + var2 + parameters->dig_p7) >> 4));
	} else {
		pressure = 0;
	}

	return pressure;
}

void BMP280_SetExampleCalibrationParameters(struct bmp280_calib_param *parameters)
{
	parameters->dig_t1 = 27504;
	parameters->dig_t2 = 26435;
	parameters->dig_t3 = -1000;
	parameters->dig_p1 = 36477;
	parameters->dig_p2 = -10685;
	parameters->dig_p3 = 3024;
	parameters->dig_p4 = 2855;
	parameters->dig_p5 = 140;
	parameters->dig_p6 = -7;
	parameters->dig_p7 = 15500;
	parameters->dig_p8 = -14600;
	parameters->dig_p9 = 6000;
	parameters->t_fine = 0;
}
