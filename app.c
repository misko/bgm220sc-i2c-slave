/*
 * I cannot hold it
 * I cannot control it
 * I'm a (I2C) slave 4 U
 * I won't deny it
 * I'm not trying to hide it
 *      - Britney Spears
 */

#include <stdbool.h>
#include <stddef.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_assert.h"
#include "em_device.h"

// Defines
#define I2C_ADDRESS                     0x30 //E2
#define I2C_BUFFER_SIZE                 100

// Buffers
uint8_t i2c_Buffer[I2C_BUFFER_SIZE];
uint8_t i2c_BufferIndex;

// Transmission flags
volatile bool i2c_gotTargetAddress;
volatile bool i2c_rxInProgress;

void initI2C(void);

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void) {

	CMU_ClockEnable(cmuClock_I2C1, true);
	// Setting up i2c
	initI2C();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void) {

	/*while (1) {
		// Receiving I2C data; keep in EM1 during transmission
		while (i2c_rxInProgress) {
			//EMU_EnterEM1();
		}
	}*/
}



/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void initI2C(void) {
	// Using default settings
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	// Configure to be addressable as slave
	i2cInit.master = false;

	// Using PA5 (SDA) and PA6 (SCL)
	GPIO_PinModeSet(gpioPortD, 2, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortD, 3, gpioModeWiredAndPullUpFilter, 1);

	// Enable pins at location 15 as specified in datasheet
	GPIO->I2CROUTE[1].SDAROUTE = (GPIO->I2CROUTE[0].SDAROUTE
			& ~_GPIO_I2C_SDAROUTE_MASK)
			| (gpioPortD << _GPIO_I2C_SDAROUTE_PORT_SHIFT
					| (2 << _GPIO_I2C_SDAROUTE_PIN_SHIFT));
	GPIO->I2CROUTE[1].SCLROUTE = (GPIO->I2CROUTE[0].SCLROUTE
			& ~_GPIO_I2C_SCLROUTE_MASK)
			| (gpioPortD << _GPIO_I2C_SCLROUTE_PORT_SHIFT
					| (3 << _GPIO_I2C_SCLROUTE_PIN_SHIFT));
	GPIO->I2CROUTE[1].ROUTEEN = GPIO_I2C_ROUTEEN_SDAPEN
			| GPIO_I2C_ROUTEEN_SCLPEN;

	// Initializing the I2C
	I2C_Init(I2C1, &i2cInit);

	// Initializing the buffer index
	i2c_BufferIndex = 0;

	// Setting up to enable slave mode
	I2C_SlaveAddressSet(I2C1, I2C_ADDRESS);
	I2C_SlaveAddressMaskSet(I2C1, 0xFE); // must match exact address

	// Configure interrupts
	I2C_IntClear(I2C1, _I2C_IF_MASK);
	I2C_IntEnable(I2C1,
			I2C_IEN_ADDR | I2C_IEN_RXDATAV | I2C_IEN_ACK | I2C_IEN_SSTOP
					| I2C_IEN_BUSERR | I2C_IEN_ARBLOST);
	NVIC_EnableIRQ(I2C1_IRQn);
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler(void) {
	uint32_t pending;
	uint32_t rxData;

	pending = I2C1->IF;
	//printf("FLAGS %x\r\n\n", pending);
	/* If some sort of fault, abort transfer. */
	if (pending & (I2C_IF_BUSERR | I2C_IF_ARBLOST)) {
		i2c_rxInProgress = false;
		//GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
	} else {
		if (pending & I2C_IF_ADDR) {
			// Address Match
			// Indicating that reception is started
			rxData = I2C1->RXDATA;
			//printf("<ADDY+RXDATA:%x\r\n\n", rxData);
			I2C1->CMD = I2C_CMD_ACK;
			//printf(">ACK\r\n\n");
			i2c_rxInProgress = true;

			if (rxData & 0x1) // read bit set
					{
				if (i2c_BufferIndex < I2C_BUFFER_SIZE) {
					// transfer data
					I2C1->TXDATA = i2c_Buffer[i2c_BufferIndex++];
				} else {
					// invalid buffer index; transfer data as if slave non-responsive
					I2C1->TXDATA = 0xFF;
				}
				//printf(">TX %x\r\n\n",I2C1->TXDATA);
			} else {
				i2c_gotTargetAddress = false;
			}

			I2C_IntClear(I2C1, I2C_IF_ADDR | I2C_IF_RXDATAV);

			//GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
		} else if (pending & I2C_IF_RXDATAV) {
			rxData = I2C1->RXDATA;

			//printf("<RXDATA:%x\r\n\n", rxData);
			if (!i2c_gotTargetAddress) {
				/******************************************************/
				/* Read target address from master.                   */
				/******************************************************/
				// verify that target address is valid
				if (rxData < I2C_BUFFER_SIZE) {
					// store target address
					i2c_BufferIndex = rxData;
					I2C1->CMD = I2C_CMD_ACK;
					//printf(">ACK\r\n\n");
					i2c_gotTargetAddress = true;
				} else {
					I2C1->CMD = I2C_CMD_NACK;
					//printf(">NACK\r\n\n");
				}
			} else {
				/******************************************************/
				/* Read new data and write to target address          */
				/******************************************************/
				// verify that target address is valid
				if (i2c_BufferIndex < I2C_BUFFER_SIZE) {
					// write new data to target address; auto increment target address
					i2c_Buffer[i2c_BufferIndex++] = rxData;
					I2C1->CMD = I2C_CMD_ACK;
					//printf(">ACK\r\n\n");
				} else {
					I2C1->CMD = I2C_CMD_NACK;
					//printf(">NACK\r\n\n");
				}
			}

			I2C_IntClear(I2C1, I2C_IF_RXDATAV);
		}

		if (pending & I2C_IF_ACK) {
			/******************************************************/
			/* Master ACK'ed, so requesting more data.            */
			/******************************************************/
			if (i2c_BufferIndex < I2C_BUFFER_SIZE) {
				// transfer data
				I2C1->TXDATA = i2c_Buffer[i2c_BufferIndex++];
			} else {
				// invalid buffer index; transfer data as if slave non-responsive
				I2C1->TXDATA = 0xFF;
			}
			//printf(">TX %x\r\n\n",I2C1->TXDATA);

			I2C_IntClear(I2C1, I2C_IF_ACK);
		}

		if (pending & I2C_IF_SSTOP) {
			// end of transaction
			i2c_rxInProgress = false;

			I2C_IntClear(I2C1, I2C_IF_SSTOP);
		}
	}
}






