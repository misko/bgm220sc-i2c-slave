/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
/*****************************************************************************
 * @file i2c_master_slave.c
 * @brief I2C Demo Application
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

#include <stdbool.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"

#include "em_gpio.h"

#include <stddef.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_assert.h"
#include "em_rtc.h"
#include "em_rtcc.h"
//#include "rtcdriver.h"
//#include "sl_i2cspm.h"
//#include "sl_udelay.h"

#define T_SLAVE 3
#define T_MASTER 4
#define T_TYPE 3

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void) {
	printf("SETUP i2c\r\n\n");
	i2c_main();

	printf("DONE i2c\r\n\n");
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void) {
	//printf("WTF\r\n\n");
}

// Defines
#define I2C_ADDRESS                     0x30 //E2
#define I2C_BUFFER_SIZE                 100

// Buffers
uint8_t i2c_Buffer[I2C_BUFFER_SIZE];
uint8_t i2c_BufferIndex;

// Transmission flags
volatile bool i2c_gotTargetAddress;
volatile bool i2c_rxInProgress;

extern void disableClocks(void);

/**************************************************************************//**
 * @brief  Starting oscillators and enabling clocks
 *****************************************************************************/
void initCMU(void) {
	// Enabling clock to the I2C and GPIO
	CMU_ClockEnable(cmuClock_I2C1, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
}

/**************************************************************************//**
 * @brief GPIO initialization
 *****************************************************************************/
void initGPIO(void) {
	// Configure LED0 and LED1 as output
	//GPIO_PinModeSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortA, 6, gpioModePushPull, 0);
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
	printf("FLAGS %x\r\n\n", pending);
	/* If some sort of fault, abort transfer. */
	if (pending & (I2C_IF_BUSERR | I2C_IF_ARBLOST)) {
		i2c_rxInProgress = false;
		//GPIO_PinOutSet(BSP_GPIO_LED1_PORT, BSP_GPIO_LED1_PIN);
	} else {
		if (pending & I2C_IF_ADDR) {
			// Address Match
			// Indicating that reception is started
			rxData = I2C1->RXDATA;
			printf("<ADDY+RXDATA:%x\r\n\n", rxData);
			I2C1->CMD = I2C_CMD_ACK;
			printf(">ACK\r\n\n");
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
				printf(">TX %x\r\n\n",I2C1->TXDATA);
			} else {
				i2c_gotTargetAddress = false;
			}

			I2C_IntClear(I2C1, I2C_IF_ADDR | I2C_IF_RXDATAV);

			//GPIO_PinOutSet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN);
		} else if (pending & I2C_IF_RXDATAV) {
			rxData = I2C1->RXDATA;

			printf("<RXDATA:%x\r\n\n", rxData);
			if (!i2c_gotTargetAddress) {
				/******************************************************/
				/* Read target address from master.                   */
				/******************************************************/
				// verify that target address is valid
				if (rxData < I2C_BUFFER_SIZE) {
					// store target address
					i2c_BufferIndex = rxData;
					I2C1->CMD = I2C_CMD_ACK;
					printf(">ACK\r\n\n");
					i2c_gotTargetAddress = true;
				} else {
					I2C1->CMD = I2C_CMD_NACK;
					printf(">NACK\r\n\n");
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
					printf(">ACK\r\n\n");
				} else {
					I2C1->CMD = I2C_CMD_NACK;
					printf(">NACK\r\n\n");
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
			printf(">TX %x\r\n\n",I2C1->TXDATA);

			I2C_IntClear(I2C1, I2C_IF_ACK);
		}

		if (pending & I2C_IF_SSTOP) {
			// end of transaction
			i2c_rxInProgress = false;

			I2C_IntClear(I2C1, I2C_IF_SSTOP);
		}
	}
}

/***************************************************************************//**
 * @brief
 *   Enter EM2 with RTCC running on a low frequency oscillator.
 *
 * @param[in] osc
 *   Oscillator to run RTCC from (LFXO or LFRCO).
 * @param[in] powerdownRam
 *   Power down all RAM except the first 16 kB block or retain full RAM.
 *
 * @details
 *   Parameter:
 *     EM2. Deep Sleep Mode.@n
 *   Condition:
 *     RTCC, 32.768 kHz LFXO or LFRCO.@n
 *
 * @note
 *   To better understand disabling clocks and oscillators for specific modes,
 *   see Reference Manual section EMU-Energy Management Unit and Table 9.2.
 ******************************************************************************/
void em_EM2_RTCC(CMU_Select_TypeDef osc, bool powerdownRam) {
	// Make sure clocks are disabled.
	disableClocks();

	// Route desired oscillator to RTCC clock tree.
	CMU_ClockSelectSet(cmuClock_RTCCCLK, osc);

	// Setup RTC parameters
	RTCC_Init_TypeDef rtccInit = RTCC_INIT_DEFAULT;
	rtccInit.presc = rtccCntPresc_1;

	// Initialize RTCC
	CMU_ClockEnable(cmuClock_RTCC, true);
	RTCC_Reset();
	RTCC_Init(&rtccInit);

	// Power down all RAM blocks except block 0
	//if (powerdownRam) {
	//EMU_RamPowerDown(SRAM_BASE, 0);
	//}

	// Enter EM2.
	EMU_EnterEM2(true);
}

/**************************************************************************//**
 * @brief  Main function
 *****************************************************************************/
int i2c_main(void) {
	// Chip errata
	CHIP_Init();

	// Initializations
	initCMU();
	initGPIO();

	// Setting up i2c
	initI2C();

	while (1) {
		// Receiving I2C data; keep in EM1 during transmission
		while (i2c_rxInProgress) {
			EMU_EnterEM1();
		}

		// EM2 entry is a critical section and interrupts are disabled to prevent
		// race conditions
		//CORE_DECLARE_IRQ_STATE;
		//CORE_ENTER_CRITICAL();
		//GPIO_PinOutClear(gpioPortA, 6);

		// Enter EM2. The I2C address match will wake up the EFM32
		// EM2 with RTCC running off LFRCO is a documented current mode in the DS
		//em_EM2_RTCC(cmuSelect_LFRCO, false);
		//CORE_EXIT_CRITICAL();
	}
}

#include "em_device.h"
#include "em_cmu.h"

/***************************************************************************//**
 * @brief   Disable high frequency clocks
 ******************************************************************************/
static void disableHFClocks(void) {
	// Disable high frequency peripherals
	CMU_ClockEnable(cmuClock_TIMER0, false);
	CMU_ClockEnable(cmuClock_TIMER1, false);
	CMU_ClockEnable(cmuClock_TIMER2, false);
	CMU_ClockEnable(cmuClock_TIMER3, false);
	CMU_ClockEnable(cmuClock_TIMER4, false);
	CMU_ClockEnable(cmuClock_PDM, false);
	CMU_ClockEnable(cmuClock_EUART0, false);
	CMU_ClockEnable(cmuClock_IADC0, false);
}

/***************************************************************************//**
 * @brief   Disable low frequency clocks
 ******************************************************************************/
static void disableLFClocks(void) {
	// Disable low frequency peripherals
	CMU_ClockEnable(cmuClock_LETIMER0, false);
	CMU_ClockEnable(cmuClock_WDOG0, false);
	CMU_ClockEnable(cmuClock_RTCC, false);
	CMU_ClockEnable(cmuClock_BURTC, false);
}

/***************************************************************************//**
 * @brief   Disable all clocks to achieve lowest current consumption numbers.
 ******************************************************************************/
extern void disableClocks(void) {
	// Disable High Frequency Clocks
	//disableHFClocks();

	// Disable Low Frequency Clocks
	//disableLFClocks();
}
