/*****************************************************************************
 *   pmu.c:  Power Management Unit(PMU) file for NXP LPC13xx 
 *   Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.07.20  ver 1.00    Prelimnary version, first Release
 *
*****************************************************************************/
#include "LPC13xx.h"			/* LPC13xx Peripheral Registers */
#include "type.h"
#include "gpio.h"
#include "pmu.h"

volatile uint32_t pmu_counter = 0;

/*****************************************************************************
** Function name:		WAKE_UP1_IRQHandler
**
** Descriptions:		WAKEUP1 Interrupt Handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WAKE_UP1_IRQHandler(void)
{
  uint32_t regVal;

  /* This handler takes care all the port pins if they
  are configured as wakeup source. */
  regVal = LPC_SYSCON->STARTSRP0;
  if ( regVal != 0 )
  {
	LPC_SYSCON->STARTRSRP0CLR = regVal;
  }
  regVal = LPC_SYSCON->STARTSRP1;
  if ( regVal != 0 )
  {
	LPC_SYSCON->STARTRSRP1CLR = regVal;
  }
  /* See tracker for bug report. */
  __NOP();
  return;
}

/*****************************************************************************
** Function name:		PMU_Init
**
** Descriptions:		Initialize PMU and setup wakeup source.
**						For Sleep and deepsleep, any of the I/O pins can be 
**						used as the wakeup source.
**						For Deep Powerdown, only pin P1.4 can be used as 
**						wakeup source from deep powerdown. 
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void PMU_Init( void )
{
  /* Enable all clocks, even those turned off at power up. */
  LPC_SYSCON->PDRUNCFG &= ~(WDT_OSC_PD|SYS_OSC_PD|ADC_PD);

  NVIC_EnableIRQ((IRQn_Type)(WAKEUP1_IRQn));
  
  /* use port0_1 as wakeup source, i/o pin */
  LPC_IOCON->PIO0_1 &= ~0x07;	
  LPC_IOCON->PIO0_1 |= 0x20;	
  GPIOSetDir( PORT0, 1, 0 );	/* Input P0.1 */
  /* Only edge trigger. activation polarity on P0.1 is FALLING EDGE. */
  LPC_SYSCON->STARTAPRP0 = 0x00000000;
  /* Clear all wakeup source */ 
  LPC_SYSCON->STARTRSRP0CLR = 0xFFFFFFFF;
  LPC_SYSCON->STARTRSRP1CLR = 0xFFFFFFFF;
  /* Enable Port 0.1 as wakeup source. */
  LPC_SYSCON->STARTERP0 = 0x1<<1;
  return;
}

/*****************************************************************************
** Function name:		PMU_Sleep
**
** Descriptions:		Put some of the peripheral in sleep mode.
**
** parameters:			SleepMode: 1 is deep sleep, 0 is sleep, 
**						Sleep peripheral module(s)
** Returned value:		None
** 
*****************************************************************************/
void PMU_Sleep( uint32_t SleepMode, uint32_t SleepCtrl )
{
  LPC_SYSCON->PDAWAKECFG = LPC_SYSCON->PDRUNCFG;
  LPC_SYSCON->PDSLEEPCFG = SleepCtrl;
  if ( SleepMode )
  {
	SCB->SCR |= NVIC_LP_SLEEPDEEP;
  }
  __WFI();
  return;
}

/*****************************************************************************
** Function name:		PMU_PowerDown
**
** Descriptions:		Some of the content should not be touched 
**						during the power down to wakeup process.
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void PMU_PowerDown( void )
{
  uint32_t regVal;

  if ( (LPC_PMU->PCON & ((0x1<<8) | (0x1<<11))) != 0x0 )
  {
	/* Check sleep and deep power down bits. If sleep and/or
	deep power down mode are entered, clear the PCON bits. */
    regVal = LPC_PMU->PCON;
	regVal |= ((0x1<<8) | (0x1<<11));
    LPC_PMU->PCON = regVal;

    if ( (LPC_PMU->GPREG0 != 0x12345678)||(LPC_PMU->GPREG1 != 0x87654321)
		||(LPC_PMU->GPREG2 != 0x56781234)||(LPC_PMU->GPREG3 != 0x43218765) )
    {
      while (1);
    }
  }
  else
  {
	/* If in neither sleep nor deep power mode, enter deep power
	down mode now. */
    LPC_PMU->GPREG0 = 0x12345678;
    LPC_PMU->GPREG1 = 0x87654321;
    LPC_PMU->GPREG2 = 0x56781234;
    LPC_PMU->GPREG3 = 0x43218765;
	SCB->SCR |= NVIC_LP_SLEEPDEEP;
    LPC_PMU->PCON = 0x2;
    __WFI();
  }
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
