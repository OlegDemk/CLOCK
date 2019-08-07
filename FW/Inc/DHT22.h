#ifndef DHT22_H_
#define DHT22_H_
//--------------------------------------------------
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

void port_init(void);
uint8_t DHD22_Reset(void);
//--------------------------------------------------
#endif /* DHT22_H_ */