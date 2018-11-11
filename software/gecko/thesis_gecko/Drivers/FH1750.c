/*
 * FH1750.c
 *
 *  Created on: Nov 7, 2018
 *      Author: sebokbence
 */

#include "FH1750.h"
#include "stdbool.h"
#include "em_i2c.h"

extern uint32_t rxBuffer[100];
extern uint8_t rxIndex;
extern volatile uint8_t BH1750_MODE;
extern volatile uint8_t BH1750_MTreg;
extern const float BH1750_CONV_FACTOR;

/**
* Configure BH1750 with specified mode
* @param mode Measurement mode
*/
int BH1750_Configure(Mode mode) {

    // default transmission result to a value out of normal range
    uint32_t result = 5;
    uint16_t data = 0;

    // Check measurement mode is valid
    switch (mode) {

      case CONTINUOUS_HIGH_RES_MODE:
      case CONTINUOUS_HIGH_RES_MODE_2:
      case CONTINUOUS_LOW_RES_MODE:
      case ONE_TIME_HIGH_RES_MODE:
      case ONE_TIME_HIGH_RES_MODE_2:
      case ONE_TIME_LOW_RES_MODE:
		// Send mode to sensor
		BH1750_I2CWriteByte(I2C0, BH1750_ADDRESS << 1, (uint8_t)mode);
		// Wait a few moments to wake up
		delay_ms(1000);
		int size = 0;
		size = BH1750_ReadData(I2C0, BH1750_ADDRESS << 1, &data);
		break;
      default:
        // Invalid measurement mode
        break;
    }

    // Check result code
    switch (result) {
      case 0:
      	BH1750_MODE = mode;
        return true;
      case 1: // too long for transmit buffer
        break;
      case 2: // received NACK on transmit of address
        break;
      case 3: // received NACK on transmit of data
        break;
      case 4: // other error
        break;
      default:
        break;
    }
    return false;
  }

  /**************************************************************************//**
   * @brief
   *  Reads data from the Si7013 sensor.
   * @param[in] i2c
   *   The I2C peripheral to use (not used).
   * @param[in] addr
   *   The I2C address of the sensor.
   * @param[out] data
   *   The data read from the sensor.
   * @return
   *   Returns number of bytes read on success. Otherwise returns error codes
   *   based on the I2CDRV.
   *****************************************************************************/
    int32_t BH1750_ReadData(I2C_TypeDef *i2c, uint8_t addr, uint16_t *data)
  {
    I2C_TransferSeq_TypeDef    seq;
    I2C_TransferReturn_TypeDef ret;
    uint8_t                    i2c_read_data[2];

    seq.addr  = addr;
    seq.flags = I2C_FLAG_READ;
    /* Select command to issue */
    seq.buf[0].data = i2c_read_data;
    seq.buf[0].len  = 2;
    /* Select location/length of data to be read */
    seq.buf[1].data = i2c_read_data;
    seq.buf[1].len  = 2;

    ret = I2C_TransferInit(i2c, &seq);
    	// Sending data
    	while (ret == i2cTransferInProgress)
    	{
    		ret = I2C_Transfer(i2c);
    	}

    	*data = ((uint16_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);
    	return i2cTransferDone;
  }

void delay_ms(int ms)
{
	int i;
	for(i = 0; i < ms; i++) { UDELAY_Delay(1000); }
}


    int32_t BH1750_I2CTransmit(I2C_TypeDef *i2c, uint8_t addr, uint32_t *data,
                                  uint8_t command)
    {
      I2C_TransferSeq_TypeDef    seq;
      I2C_TransferReturn_TypeDef ret;
      uint8_t                    i2c_read_data[2];
      uint8_t                    i2c_write_data[1];

      seq.addr  = addr;
      seq.flags = I2C_FLAG_WRITE_READ;
      /* Select command to issue */
      i2c_write_data[0] = command;
      seq.buf[0].data   = i2c_write_data;
      seq.buf[0].len    = 1;
      /* Select location/length of data to be read */
      seq.buf[1].data = i2c_read_data;
      seq.buf[1].len  = 2;

      ret = I2C_Transfer(i2c);

      if (ret != i2cTransferDone) {
        *data = 0;
        return((int) ret);
      }

      *data = ((uint32_t) i2c_read_data[0] << 8) + (i2c_read_data[1] & 0xfc);

      return((int) 2);
    }

    int32_t BH1750_I2CWriteByte(I2C_TypeDef *i2c, uint8_t addr, uint8_t command)
    {
      I2C_TransferSeq_TypeDef    seq;
      I2C_TransferReturn_TypeDef ret;
      uint8_t                    i2c_write_data[1];

      seq.addr  = addr;
      seq.flags = I2C_FLAG_WRITE;
      /* Select command to issue */
      i2c_write_data[0] = command;
      seq.buf[0].data   = i2c_write_data;
      seq.buf[0].len    = 1;

      ret = I2C_TransferInit(i2c, &seq);
      	// Sending data
      	while (ret == i2cTransferInProgress)
      	{
      		ret = I2C_Transfer(i2c);
      	}

      	return i2cTransferDone;
    }

  /**
   * Read light level from sensor
   * The return value range differs if the MTreg value is changed. The global
   * maximum value is noted in the square brackets.
   * @return Light level in lux (0.0 ~ 54612,5 [117758,203])
   * 	   -1 : no valid return value
   * 	   -2 : sensor not configured
   */
    float BH1750_readLightLevel(int maxWait, uint16_t *lightLevel) {

  	uint32_t result = 0;

    if (BH1750_MODE == UNCONFIGURED) {
      return -2.0;
    }

    // Measurement result will be stored here
    float level = -1.0;

    // Send mode to sensor
    BH1750_I2CWriteByte(I2C0, BH1750_ADDRESS << 1, (uint8_t)BH1750_MODE);

    // Wait for measurement to be taken.
    // Measurements have a maximum measurement time and a typical measurement
    // time. The maxWait argument determines which measurement wait time is
    // used when a one-time mode is being used. The typical (shorter)
    // measurement time is used by default and if maxWait is set to True then
    // the maximum measurement time will be used. See data sheet pages 2, 5
    // and 7 for more details.
    // A continuous mode measurement can be read immediately after re-sending
    // the mode command.

    switch (BH1750_MODE) {

      case ONE_TIME_LOW_RES_MODE:
        maxWait ? delay_ms(24 * BH1750_MTreg/(byte)BH1750_DEFAULT_MTREG) : delay_ms(16 * BH1750_MTreg/(byte)BH1750_DEFAULT_MTREG);
        break;
      case ONE_TIME_HIGH_RES_MODE:
      case ONE_TIME_HIGH_RES_MODE_2:
        maxWait ? delay_ms(180 * BH1750_MTreg/(byte)BH1750_DEFAULT_MTREG) : delay_ms(120 * BH1750_MTreg/(byte)BH1750_DEFAULT_MTREG);
        break;
      default:
        break;
    }

    // Read two bytes from the sensor, which are low and high parts of the sensor
    // value
    int size = 0;
    size = BH1750_ReadData(I2C0, BH1750_ADDRESS << 1, lightLevel);
    if (2 == size) {
      unsigned int tmp = 0;
      tmp = rxBuffer[rxIndex - 1];
      tmp <<= 8;
      tmp |= rxBuffer[rxIndex];
      level = tmp;
    }

    if (level != -1.0) {
      if (BH1750_MTreg != BH1750_DEFAULT_MTREG) {
       level *= (float)((byte)BH1750_DEFAULT_MTREG/(float)BH1750_MTreg);
       // Print MTreg factor if debug enabled
      }
      if (BH1750_MODE == ONE_TIME_HIGH_RES_MODE_2 || BH1750_MODE == CONTINUOUS_HIGH_RES_MODE_2) {
        level /= 2;
      }
      // Convert raw value to lux
      level /= BH1750_CONV_FACTOR;
  }
    return level;
}
