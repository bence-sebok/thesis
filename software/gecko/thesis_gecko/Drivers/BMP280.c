/*
 * KX126.c
 *
 *  Created on: Nov 5, 2018
 *      Author: sebokbence
 */

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
	  I2C0->SADDR = I2C_ADDRESS_7BIT_KX126;
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


/**
 * @brief
 *  Writes data to KX126 sensor. Write a single byte (data) to the register pointed by registerAddress.
 * @param[in] registerAddress
 *   The register to write to.
 * @param[in] data
 *   Byte to write to the register.
 * @return
 *   Status of I2C transfer.
*/
uint16_t KX126_I2C_SendCommand(uint8_t command)
{
	// Local variables for I2C transfer
	I2C_TransferSeq_TypeDef    seq;
	I2C_TransferReturn_TypeDef ret;
	uint8_t commands[2] = {0};
	uint16_t commandSize = 0;
	uint8_t bufferRead[2] = {0};
	uint16_t sizeRead = 2;
	uint16_t retval = 0;

	switch(command)
	{
	case SI7051_COMMAND_READ_FIRMWARE_VERSION:
		commands[0] = 0x84;
		commands[1] = 0xB8;
		commandSize = 2;
		break;
	case SI7051_COMMAND_MEASUREMENT_NOHOLD:
		commands[0] = 0xE3;
		commandSize = 1;
		break;
	case SI7051_COMMAND_MEASUREMENT_HOLD:
		commands[0] = 0xF3;
		commandSize = 1;
		break;
	case SI7051_COMMAND_READ_USERREGISTER1:
		commands[0] = 0xE7;
		commandSize = 1;
		break;
	case SI7051_COMMAND_RESET:
			commands[0] = 0xFE;
			commandSize = 1;
			break;
	default:
		return 0;
	}

	// Set seq values
	seq.addr  = I2C_ADDRESS_8BIT_SI7051;
	seq.flags = I2C_FLAG_WRITE_READ;
	seq.buf[0].data   = commands;
	seq.buf[0].len    = commandSize;
	/* Select location/length of data to be read */
	seq.buf[1].data = (uint8_t *)bufferRead;
	seq.buf[1].len  = sizeRead;

	ret = I2C_TransferInit(SI7051_I2C, &seq);
	// Sending data
	while (ret == i2cTransferInProgress)
	{
		ret = I2C_Transfer(SI7051_I2C);
	}

	switch(command)
	{
	case SI7051_COMMAND_READ_FIRMWARE_VERSION:
		memcpy(&retval, bufferRead, 2);
		break;
	case SI7051_COMMAND_MEASUREMENT_NOHOLD:
		retval = (bufferRead[0] << 8 | bufferRead[1]);
		break;
	case SI7051_COMMAND_MEASUREMENT_HOLD:
		retval = (bufferRead[1] << 8 | bufferRead[0]);
		break;
	case SI7051_COMMAND_READ_USERREGISTER1:
		memcpy(&retval, bufferRead, 2);
		break;
	case SI7051_COMMAND_RESET:
		break;
	default:
		retval = 0;
	}

	return retval;
}

int32_t SI7051_ConvertTemperature(uint16_t Temp_Code)
{
	int32_t temperatureC = 0;
	int32_t prod = Temp_Code * 17572;
	int32_t div = prod / 6553600;
	int32_t sub = div - 4685;
	temperatureC = sub / 100;
	return temperatureC;
}

/**
 * @brief
 *  Reads X, Y, Z accelerations from sensor.
 * @return
 *   Three words of X, Y, Z accelerations.
*/
accel_data_w_t SI7051_ReadTemperature()
{
	// Local variables for XYZ data
	  int8_t readBuffer[6] = {0};
	  uint16_t readSize = 6;
	  accel_data_lh_t acceleration = {0};
	  accel_data_w_t accelerationWord = {0};
	  // Read data from sensor
	  KX126_I2C_ReadRegister(KX126_REGISTER_XOUTL, readBuffer, readSize);
	  // Return accel struct data with xyz axis data
	  memcpy(&acceleration, readBuffer, readSize);
	  memcpy(&accelerationWord, &acceleration, readSize);
	  return accelerationWord;
}

/**
 * @brief
 *  Set operating mode of sensor to full power or low power mode
 * @return
 *   none
*/
void KX126_EnableSensor(void)
{
	  uint8_t cntl1_byte = 0;
	  int8_t singleByte[1] = {0};
	  // Read CNTL1 from sensor
	  KX126_I2C_ReadRegister(KX126_REGISTER_CNTL1, singleByte, 1);
	  cntl1_byte = singleByte[0];
	  // Bit7 is PC1 (controls the operating mode)
	  // PC1 = 0 - stand-by mode
	  // PC1 = 1 - full power or low power mode
	  // Overwrite CNTL1 with PREVIOUS_VALUE | PC1
	  KX126_I2C_WriteRegister(KX126_REGISTER_CNTL1, cntl1_byte | KX126_CNTL1_PC1_SET);
}

/*
int32_t bmp280_comp_temp_32bit(uint32_t uncomp_temp)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature = 0;
	int8_t rslt;

	rslt = null_ptr_check(dev);

	if (rslt == BMP280_OK) {
		var1 = ((((uncomp_temp >> 3) - ((int32_t) dev->calib_param.dig_t1 << 1)))
		* ((int32_t) dev->calib_param.dig_t2)) >> 11;
		var2 = (((((uncomp_temp >> 4) - ((int32_t) dev->calib_param.dig_t1))
		* ((uncomp_temp >> 4) - ((int32_t) dev->calib_param.dig_t1))) >> 12)
		* ((int32_t) dev->calib_param.dig_t3)) >> 14;

		dev->calib_param.t_fine = var1 + var2;
		temperature = (dev->calib_param.t_fine * 5 + 128) >> 8;
	}

	return temperature;
}
*/
