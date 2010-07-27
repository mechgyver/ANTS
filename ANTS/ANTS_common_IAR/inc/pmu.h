/*****************************************************************************
 *   pmu.h:  Header file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.09.01  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __PMU_H 
#define __PMU_H

#define TEST_POWERDOWN		0

/* System Low Power ----------------------------------------------------------*/
#define NVIC_LP_SEVONPEND            (0x10)
#define NVIC_LP_SLEEPDEEP            (0x04)
#define NVIC_LP_SLEEPONEXIT          (0x02)

#define NUM_OF_WAKESOURCE	40
#define IRC_OUT_PD			(0x1<<0)
#define IRC_PD				(0x1<<1)
#define FLASH_PD			(0x1<<2)
#define BOD_PD				(0x1<<3)
#define ADC_PD				(0x1<<4)
#define SYS_OSC_PD			(0x1<<5)
#define WDT_OSC_PD			(0x1<<6)
#define SYS_PLL_PD			(0x1<<7)
#define USB_PLL_PD			(0x1<<8)

void WAKEUP_IRQHandler( void );
void PMU_Init( void );
void PMU_Sleep( uint32_t SleepMode, uint32_t SleepCtrl );
void PMU_PowerDown( void );

#endif /* end __PMU_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
