/***************************************************************************//**
* \brief CC2500.c header
*******************************************************************************/
#include "CC_CC2500.h"
#include "CC_spi.h"
#include "LPC13xx.h"
#include "gpio.h"
#include "stdint.h"

#define CC_RF_FREQ  2400 /** Chip carrier frequency in kHz */

void writeRFSettings(void);
void readRFSettings(void);
void RFSendPacket(uint8_t *, uint8_t);
uint8_t RFReceivePacket(uint8_t *, uint8_t *);
