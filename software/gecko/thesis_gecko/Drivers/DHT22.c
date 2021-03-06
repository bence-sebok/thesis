/*
 * DHT22.c
 *
 *  Created on: Nov 11, 2018
 *      Author: sebokbence
 */

#include <math.h>

#include "em_gpio.h"
#include "em_core.h"
#include "em_int.h"

#include "udelay.h"

#include "FH1750.h"
#include "DHT22.h"

uint32_t INT_LockCnt = 0;

// Expect the signal line to be at the specified level for a period of time and
// return a count of loop cycles spent at that level (this cycle count can be
// used to compare the relative time of two pulses).  If more than a millisecond
// ellapses without the level changing then the call fails with a 0 response.
// This is adapted from Arduino's pulseInLong function (which is only available
// in the very latest IDE versions):
//   https://github.com/arduino/Arduino/blob/master/hardware/arduino/avr/cores/arduino/wiring_pulse.c
uint32_t DHT22_ExpectPulse(unsigned int level) {
	uint32_t count = 0;
	while (GPIO_PinInGet(DHT22_DATA_PORT, DHT22_DATA_PIN) == level)
	{
	  if (count++ >= 1000)
	  {
		return 0; // Exceeded timeout, fail.
	  }
	}

	return count;
}

uint8_t DHT22_ReadSensor(dht22_data_t * output)
{
	// TODO: check the 2 seconds sensor reading interval

	// Set 40 bits of received data to zero.
	uint8_t data[5] = {0};

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  GPIO_PinOutSet(DHT22_DATA_PORT, DHT22_DATA_PIN);
  delay_ms(250);

  // First set data line low for 20 milliseconds.
  GPIO_PinModeSet(DHT22_DATA_PORT, DHT22_DATA_PIN, gpioModePushPull, 0);
  delay_ms(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
	//INT_Disable();

	// End the start signal by setting data line high for 40 microseconds.
	GPIO_PinOutSet(DHT22_DATA_PORT, DHT22_DATA_PIN);
	UDELAY_Delay(40);

	// Now start reading the data line to get the value from the DHT sensor.
	GPIO_PinModeSet(DHT22_DATA_PORT, DHT22_DATA_PIN, gpioModeInputPull, 1);
	UDELAY_Delay(10); // Delay a bit to let sensor pull data line low.

	    // First expect a low signal for ~80 microseconds followed by a high signal
	    // for ~80 microseconds again.
	    if (DHT22_ExpectPulse(0) == 0) {
	    	while(1);
	    }
	    if (DHT22_ExpectPulse(1) == 0) {
	    	while(1);
	    }

	    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
	    // microsecond low pulse followed by a variable length high pulse.  If the
	    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
	    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
	    // and use that to compare to the cycle count of the high pulse to determine
	    // if the bit is a 0 (high state cycle count < low state cycle count), or a
	    // 1 (high state cycle count > low state cycle count). Note that for speed all
	    // the pulses are read into a array and then examined in a later step.
	    for (int i=0; i < 80; i+=2) {
	      cycles[i]   = DHT22_ExpectPulse(0);
	      cycles[i+1] = DHT22_ExpectPulse(1);
	    }
	  } // Timing critical code is now complete.

  	INT_Enable();

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      while(1);
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  output->raw[0] = data[0];
  output->raw[1] = data[1];
  output->raw[2] = data[2];
  output->raw[3] = data[3];
  output->raw[4] = data[4];
  return 1;
}

void DHT22_GetTemperature(dht22_data_t *dht22_data) {
  float f = 0;
  f = dht22_data->raw[2] & 0x7F;
  f *= 256;
  f += dht22_data->raw[3];
  f *= 0.1;
  if (dht22_data->raw[2] & 0x80)
  {
	f *= -1;
  }
  dht22_data->temperature = f;
}


void DHT22_GetHumidity(dht22_data_t *dht22_data) {
	float f = 0;
	f = dht22_data->raw[0];
	f *= 256;
	f += dht22_data->raw[1];
	f *= 0.1;
	dht22_data->humidity = f;
}

float DHT22_convertCtoF(float c) {
  return c * 1.8 + 32;
}

float DHT22_convertFtoC(float f) {
  return (f - 32) * 0.55555;
}

float DHT22_ComputeHeatIndex(dht22_data_t* dht22_data) {
  // Using both Rothfusz and Steadman's equations
  // http://www.wpc.ncep.noaa.gov/html/heatindex_equation.shtml
  float hi = 0;
  float temperature = 0;

  temperature = DHT22_convertCtoF(dht22_data->temperature);

  hi = 0.5 * (temperature + 61.0 + ((temperature - 68.0) * 1.2) + (dht22_data->humidity * 0.094));

  if (hi > 79) {
    hi = -42.379 +
             2.04901523 * temperature +
            10.14333127 * dht22_data->humidity +
            -0.22475541 * temperature*dht22_data->humidity +
            -0.00683783 * pow(temperature, 2) +
            -0.05481717 * pow(dht22_data->humidity, 2) +
             0.00122874 * pow(temperature, 2) * dht22_data->humidity +
             0.00085282 * temperature*pow(dht22_data->humidity, 2) +
            -0.00000199 * pow(temperature, 2) * pow(dht22_data->humidity, 2);

    if((dht22_data->humidity < 13) && (temperature >= 80.0) && (temperature <= 112.0))
      hi -= ((13.0 - dht22_data->humidity) * 0.25) * sqrt((17.0 - abs(temperature - 95.0)) * 0.05882);

    else if((dht22_data->humidity > 85.0) && (temperature >= 80.0) && (temperature <= 87.0))
      hi += ((dht22_data->humidity - 85.0) * 0.1) * ((87.0 - temperature) * 0.2);
  }

  dht22_data->heatIndex = DHT22_convertFtoC(hi);
}
