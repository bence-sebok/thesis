/*
 * DHT22.h
 *
 *  Created on: Nov 11, 2018
 *      Author: sebokbence
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#define DHT22_DATA_PORT (gpioPortC)
#define DHT22_DATA_PIN (6U)

typedef struct {
	uint8_t raw[5];
	float temperature;
	float humidity;
	float heatIndex;
} dht22_data_t;

uint32_t DHT22_ExpectPulse(unsigned int level);
uint8_t DHT22_ReadSensor(dht22_data_t * output);
void DHT22_GetTemperature(dht22_data_t *dht22_data);
void DHT22_GetHumidity(dht22_data_t *dht22_data);
float DHT22_convertCtoF(float c);
float DHT22_convertFtoC(float f) ;
float DHT22_ComputeHeatIndex(dht22_data_t* dht22_data);

#endif /* INC_DHT22_H_ */
