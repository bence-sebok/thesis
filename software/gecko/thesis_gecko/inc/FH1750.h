/*
 * FH1750.h
 *
 *  Created on: Nov 7, 2018
 *      Author: sebokbence
 */

#ifndef INC_FH1750_H_
#define INC_FH1750_H_

#include "stdint.h"
#include "em_i2c.h"

typedef uint8_t byte;

#define BH1750_ADDRESS (0x23)
// No active state
#define BH1750_POWER_DOWN (0x00)
// Wating for measurement command
#define BH1750_POWER_ON (0x01)
// Reset data register value - not accepted in POWER_DOWN mode
#define BH1750_RESET (0x07)
// Default MTreg value
#define BH1750_DEFAULT_MTREG (0x69)
typedef enum
    {
      UNCONFIGURED = 0,
      // Measurement at 1lx resolution. Measurement time is approx 120ms.
      CONTINUOUS_HIGH_RES_MODE  = 0x10,
      // Measurement at 0.5lx resolution. Measurement time is approx 120ms.
      CONTINUOUS_HIGH_RES_MODE_2 = 0x11,
      // Measurement at 4lx resolution. Measurement time is approx 16ms.
      CONTINUOUS_LOW_RES_MODE = 0x13,
      // Measurement at 1lx resolution. Measurement time is approx 120ms.
      ONE_TIME_HIGH_RES_MODE = 0x20,
      // Measurement at 0.5lx resolution. Measurement time is approx 120ms.
      ONE_TIME_HIGH_RES_MODE_2 = 0x21,
      // Measurement at 1lx resolution. Measurement time is approx 120ms.
      ONE_TIME_LOW_RES_MODE = 0x23
    } Mode;

float BH1750_readLightLevel(int maxWait, uint16_t *lightLevel);
int BH1750_Configure(Mode mode);
int32_t BH1750_ReadData(I2C_TypeDef *i2c, uint8_t addr, uint16_t *data);
int32_t BH1750_I2CWriteByte(I2C_TypeDef *i2c, uint8_t addr, uint8_t command);
void delay_ms(int ms);

#endif /* INC_FH1750_H_ */
