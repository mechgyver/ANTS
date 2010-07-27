//------------------------------------------------------------------------------
//  Description:  Header file for TI_CC_spi.c
//  Adapted from TI example code file with the same name.
//------------------------------------------------------------------------------
#include "LPC13xx.h"
#include "ssp.h"
#include "gpio.h"
#include "CC_CC2500.h"
#include "stdint.h"

void CC_SPISetup(void);
void CC_PowerupResetCCxxxx(void);
void CC_SPIWriteReg(uint8_t, uint8_t);
void CC_SPIWriteBurstReg(uint8_t, uint8_t*, uint8_t);
char CC_SPIReadReg(uint8_t);
void CC_SPIReadBurstReg(uint8_t, uint8_t *, uint8_t);
char CC_SPIReadStatus(uint8_t);
char CC_SPIStrobe(uint8_t);
void CC_Wait(unsigned int);