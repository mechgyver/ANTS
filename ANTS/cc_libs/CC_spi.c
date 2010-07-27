//------------------------------------------------------------------------------
//  Description:  This file contains functions that allow the MCU to
//  access the SPI interface of the CC1100/CC2500.
//
//  MCU/CC1100-2500 Interface Code Library v1.1
//------------------------------------------------------------------------------
#include "CC_spi.h"
/***************************************************************************//**
*  \fn void CC_Wait(unsigned int count)
*
*  \brief Delay function.
*  \param[in] count
*  \return none
*******************************************************************************/
void CC_Wait(unsigned int count)
{
  while(count > 0)
  count--;
}
/***************************************************************************//**
*  \fn void CC_SPIWriteReg(char addr, char value)
*
*  \brief Writes "value" to a single configuration register at address "addr".
*  \param[in] addr
*  \param[in] value
*  \return none
*******************************************************************************/
void CC_SPIWriteReg(uint8_t addr, uint8_t value)
{
    uint8_t tmp;
  //  CC_CSn_PxOUT &= ~CC_CSn_PIN;        // /CS enable
//  while (!(IFG1&UTXIFG0));                  // Wait for TX to finish
//  U0TXBUF = addr;                           // Send address
//  while (!(IFG1&UTXIFG0));                  // Wait for TX to finish
//  U0TXBUF = value;                          // Send value
//  while(!(UTCTL0&TXEPT));                   // Wait for TX complete
//  CC_CSn_PxOUT |= CC_CSn_PIN;         // /CS disable
  
    GPIOSetValue( PORT0, 2, 0 ); // /CS enable
    while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish
    LPC_SSP->DR = addr; // Send address    
    while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
    tmp = LPC_SSP->DR;    
    while( LPC_SSP->SR & SSPSR_BSY ); // Wait until the Busy bit is cleared
    while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish
    LPC_SSP->DR = value; // Send value
    while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
    tmp = LPC_SSP->DR;
    GPIOSetValue( PORT0, 2, 1 ); // /CS disable  
}

/***************************************************************************//**
*  \fn void CC_SPIWriteBurstReg(char addr, char *buffer, char count)
*
*  \brief Writes values to multiple configuration registers, the first register being
*  at address "addr".  First data byte is at "buffer", and both addr and
*  buffer are incremented sequentially (within the CCxxxx and MCU,
*  respectively) until "count" writes have been performed.
*  \param[in] addr
*  \param[in] *buffer
*  \param[in] count
*  \return none
*******************************************************************************/
void CC_SPIWriteBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
  char i;

//  CC_CSn_PxOUT &= ~CC_CSn_PIN;        // /CS enable
//  while (!(IFG1 & UTXIFG0));                // Wait for TX to finish
//  U0TXBUF = addr | CCxxx0_WRITE_BURST;   // Send address
//  for (i = 0; i < count; i++)
//  {
//    while (!(IFG1 & UTXIFG0));              // Wait for TX to finish
//    U0TXBUF = buffer[i];                    // Send data
//  }
//  while(!(UTCTL0 & TXEPT));
//  CC_CSn_PxOUT |= CC_CSn_PIN;         // /CS disable

  GPIOSetValue( PORT0, 2, 0 );        // /CS enable
  addr |= CCxxx0_WRITE_BURST; //enable Burst bit
  LPC_SSP->DR = addr;
  for (i = 0; i < count; i++)
  {
    while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish
    LPC_SSP->DR = buffer[i]; // Send data
  }
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF );
  GPIOSetValue( PORT0, 2, 1 );         // /CS disable
}
/***************************************************************************//**
*  \fn char CC_SPIReadReg(char addr)
*  \brief Reads a single configuration register at address "addr" and returns the value read.
*  \param[in] addr Register address.
*  \return char Register value.
*******************************************************************************/
char CC_SPIReadReg(uint8_t addr)
{
  char x;
  GPIOSetValue( PORT0, 2, 0 );        // /CS enable     
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = (addr | CCxxx0_READ_SINGLE); // Send address
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  x = LPC_SSP->DR; // clear data register
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = 0x00; // Dummy write so we can read data
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  x = LPC_SSP->DR; // Read data
  GPIOSetValue( PORT0, 2, 1 );        // /CS disable
  return x;
}
/**************************************************************************//**
*  \fn void CC_SPIReadBurstReg(char addr, char *buffer, char count)
*
*  \brief Reads multiple configuration registers, the first register being at address
*  "addr".  Values read are deposited sequentially starting at address
*  "buffer", until "count" registers have been read.
*  \param[in] addr Register address.
*  \param[in] *buffer Pointer to buffer that will contain register values.
*  \param[in] count Number of bytes to read.
*  \return none
*******************************************************************************/
void CC_SPIReadBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
{
  unsigned int i;

//  CC_CSn_PxOUT &= ~CC_CSn_PIN;        // /CS enable
//  while (!(IFG1 & UTXIFG0));                // Wait for TXBUF ready
//  U0TXBUF = (addr | CCxxx0_READ_BURST);  // Send address
//  while(!(UTCTL0 & TXEPT));                 // Wait for TX complete
//  U0TXBUF = 0;                              // Dummy write to read 1st data byte
//  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
//  IFG1 &= ~URXIFG0;                         // Clear flag
//  while (!(IFG1&URXIFG0));                  // Wait for end of 1st data byte TX
//  // First data byte now in RXBUF
//  for (i = 0; i < (count-1); i++)
//  {
//    U0TXBUF = 0;                            // Initiate next data RX, meanwhile
//    buffer[i] = U0RXBUF;                    // Store data from last data RX
//    while (!(IFG1&URXIFG0));                // Wait for end of data RX
//  }
//  buffer[count-1] = U0RXBUF;                // Store last RX byte in buffer
//  CC_CSn_PxOUT |= CC_CSn_PIN;         // /CS disable
  
  GPIOSetValue( PORT0, 2, 0 );        // /CS enable
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = (addr | CCxxx0_READ_BURST); // Send address
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = 0x00; // Dummy write so we can read first data byte
  // Addr byte is now being TX'ed, with dummy byte to follow immediately after
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared  // First data byte now in RXBUF
  for (i = 0; i < (count-1); i++)
  {
    LPC_SSP->DR = 0x00; // Initiate next data RX, meanwhile
    buffer[i] = LPC_SSP->DR; // Store data from last data RX
    while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  }
  buffer[count-1] = LPC_SSP->DR; // Store last RX byte in buffer
  GPIOSetValue( PORT0, 2, 1 ); // /CS disable
}
// For status/strobe addresses, the BURST bit selects between status registers
// and command strobes.
/***************************************************************************//**
*  \fn char CC_SPIReadStatus(char addr)
*
*  \brief Special read function for reading status registers.  Reads status register
*  at register "addr" and returns the value read.
*  \param[in] addr Status register address.
*  \return char Status register byte.
*******************************************************************************/
char CC_SPIReadStatus(uint8_t addr)
{
  char status;

  GPIOSetValue( PORT0, 2, 0 ); // /CS enable
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = (addr | CCxxx0_READ_BURST);  // Send address
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  status = LPC_SSP->DR; // clear SSP Data register
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = 0x00;// Dummy write so we can read data
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  status = LPC_SSP->DR; // Read data
  GPIOSetValue( PORT0, 2, 1 ); // /CS disable

  return status;
}
/***************************************************************************//**
*  \fn char CC_SPIStrobe(char strobe)
*
*  \brief Special write function for writing to command strobe registers.  
*   Writes to the strobe at address "addr".
*  \param[in] strobe Strobe register address.
*  \return status
*******************************************************************************/
char CC_SPIStrobe(uint8_t strobe)
{
  char status;
  
  GPIOSetValue( PORT0, 2, 0 ); // /CS enable
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = strobe; // Send strobe
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  status = LPC_SSP->DR; // clear SSP Data register
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = 0x00;// Dummy write so we can read data
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE ); // Wait until the Busy bit is cleared
  status = LPC_SSP->DR; // Read data
  //while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
  GPIOSetValue( PORT0, 2, 1 ); // /CS disable
  
  return status;
}

/***************************************************************************//**
*  \fn void CC_PowerupResetCCxxxx(void)
*  \brief Resets registers to default values.
*  \param[in] none
*  \param[out] none
*  \return none
*******************************************************************************/
void CC_PowerupResetCCxxxx(void)
{
  unsigned char Dummy;
  GPIOSetValue( PORT0, 2, 1 );        // /CS disable
  CC_Wait(30); //30
  GPIOSetValue( PORT0, 2, 0 );        // /CS enable
  CC_Wait(30); //30
  GPIOSetValue( PORT0, 2, 1 );        // /CS disable
  CC_Wait(45); //45

  GPIOSetValue( PORT0, 2, 0 );        // /CS enable
  while ( (LPC_SSP->SR & (SSPSR_TNF|SSPSR_BSY)) != SSPSR_TNF ); // Wait for TX to finish and make sure FIFO isn't full
  LPC_SSP->DR = CCxxx0_SRES;                 // Send strobe

  // Strobe addr is now being TX'ed
  while ( (LPC_SSP->SR & (SSPSR_BSY|SSPSR_RNE)) != SSPSR_RNE );
  CC_Wait(1500); 
  Dummy = LPC_SSP->DR; // clear SSP data register
  GPIOSetValue( PORT0, 2, 1 );        // /CS disable

}
//******************************************************************************