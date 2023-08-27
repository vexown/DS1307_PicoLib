#ifndef DS1307_H
#define DS1307_H

#include "stdint.h"

int setupPinsI2C0();
uint8_t SetCurrentDate(const char *buildDate, const char *buildTime);
uint8_t ConvertBCD(uint16_t valueToConvert, bool direction);
int getMonthNumber(const char *monthAbbreviation);
uint8_t Enable_DS1307_Oscillator();
uint8_t Disable_DS1307_SquareWaveOutput();

#define INCORRECT_MONTH (0xFF)
#define INCORRECT_REQUEST (0xFF)
#define BCD_TO_DEC (bool)0
#define DEC_TO_BCD (bool)1
#define LOWER_NIBBLE_MASK 0x0F
#define CH_BIT_REG_0_READ_MASK 0x80
#define CH_BIT_REG_0_CLEAR_MASK 0x7F

#endif /* DS1307_H */
