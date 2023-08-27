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

i2c_hw_t *I2C0_Regs = I2C0_REGISTER_STRUCTURE;
resets_hw_t *ResetCtrl_Regs = RESET_CONTROL_REGISTER_STRUCTURE;

void Reset_I2C0()
{
	hw_set_bits(&ResetCtrl_Regs->reset, RESETS_RESET_I2C0_BITS);
	hw_clear_bits(&ResetCtrl_Regs->reset, RESETS_RESET_I2C0_BITS);

	/* Wait for the module to get out of the reset state */
    while (~ResetCtrl_Regs->reset_done & RESETS_RESET_I2C0_BITS)
	{
		tight_loop_contents();
	}
}

/* Perform initial configuration according to 4.3.10.2.1 and 4.3.14 Datasheet chapters */
void I2C_Initialize(uint32_t baudrate) 
{
	/* Disable the DW_apb_i2c device - only then it can be configured */
    I2C0_Regs->enable = 0;

	/* Configure I2C0 with the following options: 
	 * MASTER, FAST MODE, 7-BIT ADDRESSING, RESTART COND ENABLED, DEFAULT TX_EMPTY INTERRUPT */
    I2C0_Regs->con =
			(
				(I2C_IC_CON_MASTER_MODE_BITS | I2C_IC_CON_IC_SLAVE_DISABLE_BITS) |
				(I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB) |
				I2C_IC_CON_IC_10BITADDR_MASTER_VALUE_ADDR_7BITS |
				I2C_IC_CON_IC_RESTART_EN_BITS |
				I2C_IC_CON_TX_EMPTY_CTRL_VALUE_DISABLED |
				I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_VALUE_DISABLED
			);

	/* Frequency/Timing Configuration: (TODO - rework the timing) */ 
    /* I2C is supplied from the clk_sys clock, which is by default 125MHz (8ns period) - Datasheet 4.3.14.2 */
    uint32_t clockFreq = clock_get_hz(clk_sys); 
	/** Period of the SCL signal for I2C fast mode (1/400kHz=2.5us) is equal to 313 clk_sys cycles (approx.) 
	 *  we find that by calculating the ratio of clk_sys freq to baudrate (I2C freq) - it gives us info on 
	 *  how many clk_sys cycles there are per one baudrate cycle (baudrate/2 added to avoid division truncation) 
	*/
	uint32_t period_SCL = (clockFreq + (baudrate/2))/baudrate;

	/** Datasheet Chapter 4.3.14
	 * 	High and low counts (how long to hold high/low state of SCL during START, STOP and RESTART commands?)
	 * 
	 *  Low count value (FS_SCL_LCNT) must be larger than IC_FS_SPKLEN + 7 (Spike Suppression Limit Value) 
	 *  High count value (FS_SCL_HCNT) register values must be larger than IC_FS_SPKLEN + 5 
	 * 
	 * We divide the period_SCL (313 cycles (of clk_sys !)) between HCNT and LCNT - which makes sense, we decide 
	 * how long we hold HIGH and LOW during one I2C cycle
	 * According to I2C specification (Table 11):
	 * - min LOW period of SCL clock for Fast-mode: 0.6us (600ns)
	 * - min HIGH period of SCL clock for Fast-mode: 1.3us (1300ns)
	*/
    uint32_t lcnt = (period_SCL * 3/5); /* 187 cycles (~1500ns)  */
    uint32_t hcnt = period_SCL - lcnt; /* 126 cycles (~1000ns) */

    I2C0_Regs->fs_scl_hcnt = hcnt;
    I2C0_Regs->fs_scl_lcnt = lcnt;

	/** Spike suppression - Datasheet Chapter 4.3.11
	 *  The I2C Bus Specification calls for different maximum spike lengths (Table 10) according to the 
	 *  operating mode — 50ns for SS and FS, so this register is required to store the values needed. 
	 *  Register IC_FS_SPKLEN holds the maximum spike length for SS and FS modes (in ic_clk cycles) 
	 *  The default value for these registers is based on the value of 100ns for ic_clk period, 
	 *  so should be updated for the clk_sys period in use on RP2040
	 */
    I2C0_Regs->fs_spklen = CLK_SYS_88NS_IN_CYCLES;

#ifdef UNDERSTOOD
    // Per I2C-bus specification a device in standard or fast mode must
    // internally provide a hold time of at least 300ns for the SDA signal to
    // bridge the undefined region of the falling edge of SCL. A smaller hold
    // time of 120ns is used for fast mode plus.
    uint32_t sda_tx_hold_count;

	// sda_tx_hold_count = clockFreq [cycles/s] * 300ns * (1s / 1e9ns)
	// Reduce 300/1e9 to 3/1e7 to avoid numbers that don't fit in uint32_t.
	// Add 1 to avoid division truncation.
	sda_tx_hold_count = (((clockFreq * 3) / 10000000) + 1);

    hw_write_masked(&I2C0_Regs->sda_hold,
                    sda_tx_hold_count << I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_LSB,
                    I2C_IC_SDA_HOLD_IC_SDA_TX_HOLD_BITS);
#endif
	/* Re-enable I2C0 controller */
    I2C0_Regs->enable = 1;

}

uint8_t I2C_Register_Read(uint8_t registerAddress) 
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
    while(!i2c_write_blocking(i2c_default, DS1307_I2C_ADDRESS, &reg_address, dataToSend_length, true))
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
    while(!i2c_read_blocking(i2c_default, DS1307_I2C_ADDRESS, &reg_value, dataToRead_length, false))
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

uint8_t I2C_Register_Write(uint8_t registerAddress, uint8_t registerValue) 
{
	size_t length;
	uint32_t errorCount = 0;
	const uint32_t maxRetries = 5;
	const uint32_t retryDelayUs = 5;

    uint8_t outputData_Reset[] = {registerAddress, registerValue};
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

	return STATUS_SUCCESS;
}
