#ifndef __CRC_H_
#define __CRC_H_

#include "main.h"

uint16_t crc16(uint8_t * ptr, uint8_t DataLen);
uint8_t check_data(uint8_t * ptr, uint8_t DataLen);

#endif


