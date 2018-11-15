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

#endif /* INC_DHT22_H_ */
