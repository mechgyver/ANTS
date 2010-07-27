/*************************************************************************//**
\file timer32.c
\brief 32-bit Timer C file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.08.20  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#include "LPC13xx.h"
#include "timer32.h"
#include "gpio.h"

volatile uint32_t timer32_0_counter = 0;
volatile uint32_t timer32_1_counter = 0;
volatile uint8_t iA;

/*************************************************************************//**
** \fn delay32Ms
** \brief Start the timer delay in milliseconds until elapsed
** \param timer_num Timer number: 0 or 1
** \param delayInMs Delay value in millisecond			 					
** \return None
*****************************************************************************/
void delay32Ms(uint8_t timer_num, uint32_t delayInMs)
{
  if (timer_num == 0)
  {
    /* setup timer #0 for delay */
    LPC_TMR32B0->TCR = 0x02;		/* reset timer */
    LPC_TMR32B0->PR  = 0x00;		/* set prescaler to zero */
    LPC_TMR32B0->MR0 = delayInMs * (SystemAHBFrequency / 1000);
    LPC_TMR32B0->IR  = 0xff;		/* reset all interrrupts */
    LPC_TMR32B0->MCR = 0x04;		/* stop timer on match */
    LPC_TMR32B0->TCR = 0x01;		/* start timer */
  
    /* wait until delay time has elapsed */
    while (LPC_TMR32B0->TCR & 0x01);
  }
  else if (timer_num == 1)
  {
    /* setup timer #1 for delay */
    LPC_TMR32B1->TCR = 0x02;		/* reset timer */
    LPC_TMR32B1->PR  = 0x00;		/* set prescaler to zero */
    LPC_TMR32B1->MR0 = delayInMs * (SystemAHBFrequency / 1000);
    LPC_TMR32B1->IR  = 0xff;		/* reset all interrrupts */
    LPC_TMR32B1->MCR = 0x04;		/* stop timer on match */
    LPC_TMR32B1->TCR = 0x01;		/* start timer */
  
    /* wait until delay time has elapsed */
    while (LPC_TMR32B1->TCR & 0x01);
  }
  return;
}

/**************************************************************************//**
** \fn CT32B0_IRQHandler
** \brief Timer/Counter 0 interrupt handler
** Executes each 10ms @ 60 MHz CPU Clock
** \param None
** \return None
******************************************************************************/
void CT32B0_IRQHandler(void)
{  
  LPC_TMR32B0->IR = 1;			/* clear interrupt flag */
  timer32_0_counter++;
  switch(iA)
  {
  	case 0:
	LPC_GPIO2->DATA = 0xFFF;
	iA ^= 1;
	break;
	case 1:
	LPC_GPIO2->DATA = 0x000;
	iA ^= 1;
	break;
  }
  return;
}

/**************************************************************************//**
** \fn CT32B1_IRQHandler
** \brief Timer/Counter 1 interrupt handler executes each 10ms @ 60 MHz CPU Clock
** \param None
** \return None
******************************************************************************/
void CT32B1_IRQHandler(void)
{  
  LPC_TMR32B1->IR = 1;			/* clear interrupt flag */
  timer32_1_counter++;
  return;
}

/**************************************************************************//**
** \fn enable_timer32
** \brief Enable timer
** \param timer_num timer number: 0 or 1
** \return None
******************************************************************************/
void enable_timer32(uint8_t timer_num)
{
  if ( timer_num == 0 )
  {
    LPC_TMR32B0->TCR = 1;
  }
  else
  {
    LPC_TMR32B1->TCR = 1;
  }
  return;
}

/**************************************************************************//**
** \fn disable_timer32
** \brief Disable timer
** \param timer_num Timer number: 0 or 1
** \return None
******************************************************************************/
void disable_timer32(uint8_t timer_num)
{
  if ( timer_num == 0 )
  {
    LPC_TMR32B0->TCR = 0;
  }
  else
  {
    LPC_TMR32B1->TCR = 0;
  }
  return;
}

/**************************************************************************//**
** \fn reset_timer32
** \brief Reset timer
** \param timer number: 0 or 1
** \return None
******************************************************************************/
void reset_timer32(uint8_t timer_num)
{
  uint32_t regVal;

  if ( timer_num == 0 )
  {
    regVal = LPC_TMR32B0->TCR;
    regVal |= 0x02;
    LPC_TMR32B0->TCR = regVal;
  }
  else
  {
    regVal = LPC_TMR32B1->TCR;
    regVal |= 0x02;
    LPC_TMR32B1->TCR = regVal;
  }
  return;
}

/**************************************************************************//**
** \fn init_timer32
** \brief Initialize timer, set timer interval, reset timer, install timer interrupt handler
** \param timer_num timer number
** \param TimerInterval timer interval
** \return None
******************************************************************************/
void init_timer32(uint8_t timer_num, uint32_t TimerInterval) 
{
  if ( timer_num == 0 )
  {
    /* Some of the I/O pins need to be clearfully planned if
    you use below module because JTAG and TIMER CAP/MAT pins are muxed. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<9);
    //LPC_IOCON->PIO1_5 &= ~0x07;	/*  Timer0_32 I/O config */
    //LPC_IOCON->PIO1_5 |= 0x02;	/* Timer0_32 CAP0 */
    //LPC_IOCON->PIO1_6 &= ~0x07;
    //LPC_IOCON->PIO1_6 |= 0x02;	/* Timer0_32 MAT0 */
    //LPC_IOCON->PIO1_7 &= ~0x07;
    //LPC_IOCON->PIO1_7 |= 0x02;	/* Timer0_32 MAT1 */
    //LPC_IOCON->PIO0_1 &= ~0x07;	
    //LPC_IOCON->PIO0_1 |= 0x02;	/* Timer0_32 MAT2 */
//#ifdef __JTAG_DISABLED
//    LPC_IOCON->JTAG_TDI_PIO0_11 &= ~0x07;	
//    LPC_IOCON->JTAG_TDI_PIO0_11 |= 0x03;	/* Timer0_32 MAT3 */
//#endif

    timer32_0_counter = 0;
    disable_timer32(0);
    reset_timer32(0); //LPC_TMR32B0->TCR |= 0x02; // reset counter
    //LPC_TMR32B0->TCR &= ~0x02; // release reset
    LPC_TMR32B0->CTCR &= ~0x03; //clear CTM bits for timer to operate on every rising PCLK edge
    LPC_TMR32B0->PR = 0; // set counter prescale value
    LPC_TMR32B0->MR0 = TimerInterval; // set match register value
//	LPC_TMR32B0->EMR &= ~(0xFF<<4);
//	LPC_TMR32B0->EMR |= (0x03<<4);	/* MR0 Toggle */
    LPC_TMR32B0->MCR = 0x03; // (0b11)=configure to interrupt and Reset on MR0
    LPC_TMR32B0->IR |= 0x01; // reset MR0 interrupt
    enable_timer32(0); //LPC_TMR32B0->TCR |= 0x01; // enable counter
    
    NVIC_EnableIRQ(TIMER_32_0_IRQn); // Enable the TIMER0 Interrupt
  }
  else if ( timer_num == 1 )
  {
    /* Some of the I/O pins need to be clearfully planned if
    you use below module because JTAG and TIMER CAP/MAT pins are muxed. */
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<10); //enable clock for CT32B1
//#ifdef __JTAG_DISABLED
//    LPC_IOCON->JTAG_TMS_PIO1_0  &= ~0x07;	/*  Timer1_32 I/O config */
//    LPC_IOCON->JTAG_TMS_PIO1_0  |= 0x03;	/* Timer1_32 CAP0 */
//    LPC_IOCON->JTAG_TDO_PIO1_1  &= ~0x07;	
//    LPC_IOCON->JTAG_TDO_PIO1_1  |= 0x03;	/* Timer1_32 MAT0 */
//    LPC_IOCON->JTAG_nTRST_PIO1_2 &= ~0x07;
//    LPC_IOCON->JTAG_nTRST_PIO1_2 |= 0x03;	/* Timer1_32 MAT1 */
//    LPC_IOCON->ARM_SWDIO_PIO1_3  &= ~0x07;
//    LPC_IOCON->ARM_SWDIO_PIO1_3  |= 0x03;	/* Timer1_32 MAT2 */
//#endif
//    LPC_IOCON->PIO1_4 &= ~0x07;
//    LPC_IOCON->PIO1_4 |= 0x02;		/* Timer0_32 MAT3 */

    timer32_1_counter = 0;
    disable_timer32(1);
    reset_timer32(1);
    LPC_TMR32B1->CTCR &= ~0x03; //clear CTM bits for timer to operate on every rising PCLK edge
    LPC_TMR32B1->PR = 0; // set counter prescale value
    LPC_TMR32B1->MR0 = TimerInterval;
//	LPC_TMR32B1->EMR &= ~(0xFF<<4);
//	LPC_TMR32B1->EMR |= (0x03<<10);	/* MR3 Toggle */
    LPC_TMR32B1->MCR = 0x03; /* Interrupt and Reset on MR0 */
    LPC_TMR32B1->IR |= 0x01; // reset MR0 interrupt
    enable_timer32(1);
   
    NVIC_EnableIRQ(TIMER_32_1_IRQn); //Enable the TIMER1 Interrupt
  }
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/
