/*****************************************************************************
 *   timer16.h:  Header file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.08.20  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __TIMER16_H 
#define __TIMER16_H

#define TIME_INTERVAL	(SystemAHBFrequency/100 - 1)
/* depending on the SystemFrequency and SystemAHBFrequency setting, 
if SystemFrequency = 60Mhz, SystemAHBFrequency = 1/4 SystemAHBFrequency, 
10mSec = 150.000-1 counts */

void delayMs(uint8_t timer_num, uint32_t delayInMs);
void CT16B0_IRQHandler(void);
void CT16B1_IRQHandler(void);
void enable_timer16(uint8_t timer_num);
void disable_timer16(uint8_t timer_num);
void reset_timer16(uint8_t timer_num);
void init_timer16(uint8_t timer_num, uint32_t timerInterval);

#endif /* end __TIMER16_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
