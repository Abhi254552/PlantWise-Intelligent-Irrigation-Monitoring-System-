#ifndef __DHT11_H__
#define __DHT11_H__

#include <stdint.h>

typedef enum { DHT11_OK = 0, DHT11_ERR } DHT11_Status;

/* Initialize the DHT11 data pin (enable clock, set pin as output-high). */
void         DHT11_Init(void);
/* Read temperature and humidity; returns DHT11_OK on success. */
DHT11_Status DHT11_Read(uint8_t *temperature, uint8_t *humidity);

#endif // __DHT11_H__
