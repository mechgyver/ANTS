/*
 *  \file main.c
 *
 *  \author Adam Wahab 
 * 	\date 7/21/10.
 *
 *	\brief Main application source for ANTS firmware
 */
#include "ANTS_common_IAR/inc/system_LPC13xx.h"
#include "ANTS_common_IAR/inc/LPC13xx.h"
#include "ANTS_common_IAR/inc/gpio.h"
#include "ANTS_common_IAR/inc/ssp.h"
#include "ANTS_common_IAR/inc/i2c.h"
#include "ANTS_common_IAR/inc/timer32.h"
#include "ANTS_common_IAR/inc/timer16.h"
#include "ads_libs/ads1115.h"
#include "ANTS_common_IAR/inc/stdint.h"
#include "cc_libs/CC_spi.h"
//==============================================================================
// Defines
//==============================================================================
//#define NWK_MASTER //uncomment for access point
//#define I2C //uncomment to enable I2C at compile time
//#define ADS1115 //uncomment to enable ADS1115 drivers at compile time
#define SSP
#define CC2500
//#define UART
#define TIMER32_0
//#define TIMER32_1
//#define TIMER16_0
//#define TIMER16_1
//==============================================================================
// Prototypes
//==============================================================================
void system_init(void);

//==============================================================================
// Function definitions
//==============================================================================
void main(void)
{
	system_init(); // initialize clocks
	
	while(1)
        {
          LPC_GPIO1->DATA ^= (0xE0); //Flash LEDs
        };
}
//------------------------------------------------------------------------------
/*
 * \fn system_init	
 * \brief Initialize device specific peripherals
 */
void system_init(void)
{
	// Configure syscon (clocks)
	SystemInit();
                
	#ifdef SSP
        // Configure ssp
	SSPInit();
        #endif //#ifdef SSP

	// Configure timers
        #ifdef TIMER32_0
	init_timer32(0,2e4);
        #endif
        #ifdef TIMER32_1
        init_timer32(1, 1e4);
        #endif
        #ifdef TIMER16_0
        init_timer16(0, 2e3);
        #endif
        #ifdef TIMER16_1
        init_timer16(1, 2e4);
        #endif
	
        #ifndef NWK_MASTER	
	// Configure LED pins	
	LPC_GPIO1->DIR |= 0xE0; //PIO1_5-7 are ouputs
	LPC_GPIO1->DATA |= 0xE0; //PIO1_5-7 set high
	
	// Configure leg pins
	LPC_GPIO2->DIR |= 0xFFF; //PIO2_0-11 are outputs
	LPC_GPIO2->DATA &= ~0xFFF; //PIO2_0-11 set low
	
	// Configure HV Boost converter *SHTDN pin
	LPC_GPIO3->DIR |= 0x4; //PIO3_2 is an output
	LPC_GPIO3->DATA |= 0x4; //PIO3_2 set high
	
	#ifdef I2C
	// Configure i2c
	I2CInit(0); // !!! check to ensure 0=std mode
	// Configure ADS1115 A/D Converter
	// Configure ADS1115 alert pin
	LPC_GPIO0->DIR &= 0x80; //PIO0_7 is an input
        LPC_IOCON->PIO0_7 |= 0x10; // enable pull-up
        GPIOSetInterrupt(PORT0,0x80,0,1,1);//0,1,1=edge sense,single edge,falling
	#endif //#ifdef I2C
	#endif //#ifndef NWK_MASTER

	#ifdef NWK_MASTER
	// Configure UART
	UARTInit(9600); // !!! check to ensure valid baud
	#endif
	
        #ifdef CC2500
        // Configure CC2500 Wireless
        LPC_IOCON->PIO3_0 |= 0x10; // enable pull-up
        LPC_IOCON->PIO3_1 |= 0x10; // enable pull-up
        GPIOSetInterrupt(PORT3,0x01,0,1,1);//0,1,1=edge sense,single edge,falling
        GPIOSetInterrupt(PORT3,0x02,0,1,1);//0,1,1=edge sense,single edge,falling        
        #endif
       
  	// Configure GPIO and related interrupts
	GPIOInit();	
	// Set Interrupt Priorities
	NVIC_SetPriority(TIMER_32_0_IRQn,4);
        //NVIC_SetPriority(EINT0_IRQn,2);
	//NVIC_SetPriority(EINT3_IRQn,3);
	//NVIC_SetPriority(SSP_IRQn,1);

	// Enable Global Interrupts
        #ifdef CC2500
        GPIOIntEnable(PORT3,0); // Enable GDO0 interrupt
        GPIOIntEnable(PORT3,1); // Enable GDO1 interrupt
        #endif //CC2500
        #ifdef I2C
        #ifdef ADS1115
        GPIOIntEnable(PORT0,7); // Enable ALERT/RDY
        #endif //ADS1115
	NVIC_EnableIRQ(I2C_IRQn); // enable I2C Interrupt
        #endif //I2C
       
}
//------------------------------------------------------------------------------