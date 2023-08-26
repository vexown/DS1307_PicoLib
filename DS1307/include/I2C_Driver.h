#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#include "stdint.h"

#define I2C0_REGISTER_STRUCTURE ((i2c_hw_t *)I2C0_BASE)
#define RESET_CONTROL_REGISTER_STRUCTURE ((resets_hw_t *)RESETS_BASE)
#define I2C_FAST_MODE 400000 /* 400kHz */
#define CLK_SYS_88NS_IN_CYCLES 11
#define DS1307_I2C_ADDRESS (0x68)
#define STATUS_SUCCESS						 0
#define STAUS_FAILURE                        1
#define MPU6050_REGISTER_I2C_READ_FAIL		 ((uint8_t)0xFF)
#define MPU6050_SENSOR_DATA_READ_FAIL		 ((uint32_t)0xDEADBEEF)

uint8_t I2C_Register_Read(uint8_t registerAddress);
void I2C_Initialize(uint32_t baudrate);
void Reset_I2C0();

#endif /* I2C_DRIVER_H */
