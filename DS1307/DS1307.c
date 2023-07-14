/**
 * The DS1307 serial real-time clock (RTC) is a low power, full binary-coded decimal (BCD) clock/calendar
 * plus 56 bytes of NV SRAM. Address and data are transferred serially through an I2C, bidirectional bus.
 * The clock/calendar provides seconds, minutes, hours, day, date, month, and year information. The end of
 * the month date is automatically adjusted for months with fewer than 31 days, including corrections for leap
 * year. The clock operates in either the 24-hour or 12-hour format with AM/PM indicator. The DS1307 has a
 * built-in power-sense circuit that detects power failures and automatically switches to the backup supply.
 * 
 * Datasheet from manufacturer (Maxim): https://www.analog.com/media/en/technical-documentation/data-sheets/DS1307.pdf
 */
/**
 * DS1307 essential info:
 * - I2C Address = 0x68
 * - RTC registers: 00h to 07h. RAM registers: 08h to 3Fh. (See Table 2 in Datasheet)
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

static int DS1307_I2C_DS1307_I2C_Addressess = 0x68;

static void I2C_Register_Read(int16_t accel[3], int16_t gyro[3], int16_t *temp) 
{
	uint8_t reg_value;
    const uint8_t reg_address = registerAddress;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;
	const size_t dataToSend_length = sizeof(reg_address);
	const size_t dataToRead_length = sizeof(reg_value);
	uint32_t errorCount = 0;

	/* Send a request to read a register of the slave device */
	/* This is done with a write transaction providing the address of the register you want to read */
    while(!i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &reg_address, dataToSend_length, true))
	{
		printf("I2C write transaction failed. Retrying... "); /* Data not acknowledged by slave (or some other error) */

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}
	errorCount = 0;

	/* Send a read command specifying the number of bytes (1) you want to read (of the register address you specified in prev command) */
    while(!i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, &reg_value, dataToRead_length, false))
	{
		printf("I2C read transaction failed. Retrying... "); 

		sleep_us(retryDelayUs);
		errorCount++;
		if(errorCount > maxRetries)
		{
			return MPU6050_REGISTER_I2C_READ_FAIL;
		}
	}

	return reg_value;
}


int main() 
{
    stdio_init_all();

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    while (1) 
    {
        sleep_ms(100);
    }

    return 0;
}
