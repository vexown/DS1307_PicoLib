#ifndef DS1307_H
#define DS1307_H

#include "stdint.h"

void I2C_Initialize(uint32_t baudrate);
void Reset_I2C0();

#endif /* DS1307_H */
