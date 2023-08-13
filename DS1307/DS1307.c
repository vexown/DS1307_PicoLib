/**
 * The DS1307 serial real-time clock (RTC) is a low power, full binary-coded decimal (BCD) clock/calendar
 * plus 56 bytes of NV SRAM. Address and data are transferred serially through an I2C, bidirectional bus.
 * The clock/calendar provides seconds, minutes, hours, day, date, month, and year information. The end of
 * the month date is automatically adjusted for months with fewer than 31 days, including corrections for leap
 * year. The clock operates in either the 24-hour or 12-hour format with AM/PM indicator. The DS1307 has a
 * built-in power-sense circuit that detects power failures and automatically switches to the backup supply.
 * 
 * DS1307 Datasheet from manufacturer (Maxim): https://www.analog.com/media/en/technical-documentation/data-sheets/DS1307.pdf
 */
/**
 * DS1307 info:
 * - I2C Address = 0x68
 * - RTC registers: 00h to 07h. RAM registers: 08h to 3Fh. (See Table 2 in Datasheet)
 * 
 * MCU info:
 * - MCU used - RP2040 (Raspberry Pico board)
 * - Datasheet: https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf
 * 
 * I2C controller info:
 * - Each I2C controller is based on a configuration of the Synopsys DW_apb_i2c (v2.01) IP
 * - I2C Specification used: I2C Bus specification, version 6.0, April 2014
 * - Here is the latest I2C specification by the creator (NXP) - 2021 version: https://www.nxp.com/docs/en/user-guide/UM10204.pdf
 * 
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/regs/resets.h"
#include "hardware/address_mapped.h"
#include "hardware/resets.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "DS1307.h"
#include "I2C_Driver.h"

#define PRINTS_ENABLED 1

#if(PRINTS_ENABLED == 1)
#define LOG(...) printf(__VA_ARGS__)
#else
#define LOG(...) 
#endif

static int getMonthNumber(const char *monthAbbreviation);
static uint8_t Disable_DS1307_SquareWaveOutput();
static uint8_t Enable_DS1307_Oscillator();
static uint8_t SetCurrentDate();

static uint8_t Disable_DS1307_SquareWaveOutput() 
{
	size_t length;
	uint32_t errorCount = 0;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;

	printf("Current DS1307 SQW Status = %x \n", I2C_Register_Read(0x07));
	printf("Disabling the SQW by setting the control register (0x07) to 0x2... \n");
    uint8_t outputData_Reset[] = {0x07, 0x02};
	length = sizeof(outputData_Reset);
	while(!i2c_write_blocking(i2c_default, DS1307_I2C_ADDRESS, outputData_Reset, length, false))
	{
		printf("I2C write transaction failed. Retrying... "); /* Data not acknowledged by slave (or some other error) */

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}

	printf("Current DS1307 SQW Status = %x \n", I2C_Register_Read(0x07));
	printf("SQW disabled \n");

	return STATUS_SUCCESS;
}

/** Bit 7 of Register 0 is the clock halt (CH) bit. When this bit is set to 1, the oscillator is disabled. 
 * When cleared to 0, the oscillator is enabled. On first application of power to the device the time and 
 * date registers are typically reset to 01/01/00 01 00:00:00 (MM/DD/YY DOW HH:MM:SS). 
 * The CH bit in the seconds register will be set to a 1. The clock can be halted 
 * whenever the timekeeping functions are not required, which minimizes current (IBATDR). 
*/
static uint8_t Enable_DS1307_Oscillator() 
{
	size_t length;
	uint32_t errorCount = 0;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;

	printf("Current Oscillator Status = %x \n", I2C_Register_Read(0x00));
	printf("Enabling the Oscillator by clearing CH bit in reg 0x0... \n");
    uint8_t outputData_Reset[] = {0x00, 0x00};
	length = sizeof(outputData_Reset);
	while(!i2c_write_blocking(i2c_default, DS1307_I2C_ADDRESS, outputData_Reset, length, false))
	{
		printf("I2C write transaction failed. Retrying...  \n"); /* Data not acknowledged by slave (or some other error) */

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}

	printf("Current Oscillator Status = %x \n", I2C_Register_Read(0x00));
	printf("Oscillator enabled \n");

	/* Give the DS1307 a sec to start up */
	sleep_ms(2000); 

	return STATUS_SUCCESS;
}

static int getMonthNumber(const char *monthAbbreviation) 
{
    const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    const int monthNumbers[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    const int numMonths = sizeof(months) / sizeof(months[0]);

    for (int i = 0; i < numMonths; i++) 
	{
        if (strcmp(monthAbbreviation, months[i]) == 0) 
		{
            return monthNumbers[i];
        }
    }

    return INCORRECT_MONTH;  /* Return error value if the abbreviation is not found */
}

static uint8_t ConvertBCD(uint16_t valueToConvert, bool direction)
{
	uint8_t convertedValue;

	if(direction == DEC_TO_BCD)
	{
		convertedValue = ((((valueToConvert-(valueToConvert%10))/10) << 4) | ((valueToConvert%10)));
	}
	else if(direction == BCD_TO_DEC)
	{
		convertedValue = ((((uint8_t)(valueToConvert >> 4))*10) + ((uint8_t)(valueToConvert & LOWER_NIBBLE_MASK)));
	}
	else
	{
		convertedValue = INCORRECT_REQUEST;
	}

	return convertedValue;
}

/* Refer to Table 2. Timekeeper Registers in Datasheet to understand where the time is stored and how it's represented */
static uint8_t SetCurrentDate() 
{
	size_t length;
	uint32_t errorCount = 0;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;
	uint8_t output[2];

    const char *buildDate = __DATE__;
    const char *buildTime = __TIME__;

	char monthAbbrev[4];
    int day, month, year, hours, minutes, seconds;
    
	sscanf(buildTime, "%d:%d:%d", &hours, &minutes, &seconds);
    sscanf(buildDate, "%s %d %d", monthAbbrev, &day, &year);

	/* get only the last two numbers of the year */
	year = year%100;

    month = getMonthNumber(monthAbbrev);
    if (month == INCORRECT_MONTH) 
	{
		printf("ERROR - MONTH_NUMBER = %u \n", month);
		sleep_ms(1000);
        return STAUS_FAILURE;
    }

    uint8_t timeAndDate_au8[] = {
								(uint8_t)seconds, 
								(uint8_t)minutes, 
								(uint8_t)hours, 
								(uint8_t)0x1, /* Day of the week - I don't care about it right now */ 
								(uint8_t)day, 
								(uint8_t)month, 
								(uint8_t)year
								};

	/* Convert the values into BCD (Binary-Coded Decimal) format that DS1307 uses */
	for(uint8_t i=0; i<sizeof(timeAndDate_au8); i++)
	{
		timeAndDate_au8[i] = ConvertBCD(timeAndDate_au8[i], DEC_TO_BCD);
	}	
	
	printf("Setting current date, which is: \n");
	printf("Build Time: %x:%x:%x \n Build Date: %x/%x/%x\n", timeAndDate_au8[2], timeAndDate_au8[1], timeAndDate_au8[0],
														  	 timeAndDate_au8[4], timeAndDate_au8[5], timeAndDate_au8[6]);

    for(int reg = 0x0; reg <= 0x6; reg++) 
	{ 
		output[0] = reg;
		output[1] = timeAndDate_au8[reg];
		length = sizeof(output);
        while(!i2c_write_blocking(i2c_default, DS1307_I2C_ADDRESS, output, length, false))
		{
			printf("I2C write transaction failed. Retrying...  \n"); /* Data not acknowledged by slave (or some other error) */

			sleep_us(retryDelayUs);
			errorCount++;
			if(errorCount > maxRetries)
			{
				return MPU6050_REGISTER_I2C_READ_FAIL;
			}
		}
		sleep_ms(200);
    }

	printf("Date set \n");
	sleep_ms(200); 

	return STATUS_SUCCESS;
}

int main() 
{
    stdio_init_all();

	/* Reset the I2C0 controller to get a fresh clear state */
	Reset_I2C0();
    /* Initial Configuration of the I2C0 */
    I2C_Initialize(I2C_FAST_MODE);

	/* Configure I2C pins */
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    /* Make the I2C pins available to picotool */
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	(void)Disable_DS1307_SquareWaveOutput();
	(void)Enable_DS1307_Oscillator();
	(void)SetCurrentDate();

    while (1) 
    {
		printf("Going into I2C reg read... \n");
		for(int i = 0; i<=7; i++)
		{
			LOG("Read reg %x = 0x%x \n", i, I2C_Register_Read(i));
		}
		sleep_ms(1000);
    }

    return 0;
}

/** TODO:
 * - i2c_write_blocking function was still writing/reading via I2C even when I completely disconnected RTC module! This should not be! 
 * Find out if there is an issue with their I2C driver or maybe I misinterpreted something! If its the driver, write a better one and share with
 * the community! 
 * - Add error handling everywhere
*/