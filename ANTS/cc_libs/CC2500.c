/***************************************************************************//**
\file CC2500.c
\brief This file contains functions that configure the CC1100/2500 device.
*******************************************************************************/
#include "CC2500.h"
/*******************************************************************************
*	\fn void writeRFSettings(void)
*
*	\brief Used to configure the CCxxxx registers.  
*
*   \param[in]	none
*	\param[out] none
*	\return		none
Settings:																			  
 -Product = CC2500
 -Crystal accuracy = 40 ppm
 -X-tal frequency = 26 MHz
 -RF output power = 0 dBm
 -RX filterbandwidth = 540.000000 kHz
 -Deviation = 0.000000
 -Return state:  Return to RX state upon leaving either TX or RX
 -Datarate = 250.000000 kbps
 -Modulation = (7) MSK
 -Manchester enable = (0) Manchester disabled
 -RF Frequency = 2433.000000 MHz
 -Channel spacing = 199.950000 kHz
 -Channel number = 0
 -Optimization = Sensitivity
 -Sync mode = (3) 30/32 sync word bits detected
 -Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
 -CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
 -Forward Error Correction = (0) FEC disabled
 -Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
 -Packetlength = 255
 -Preamble count = (2)  4 bytes
 -Append status = 1
 -Address check = (0) No address check
 -FIFO autoflush = 0
 -Device address = 0
 -GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
 -GDO2 signal selection = (11) Serial Clock
*******************************************************************************/

void writeRFSettings(void)
{
    // Write register settings
    CC_SPIWriteReg(CCxxx0_IOCFG2,   0x0B); //Orig. 0x0B. GDO2 output pin config. 0x0B=0000 1011
    CC_SPIWriteReg(CCxxx0_IOCFG0,   0x06); //Orig. 0x06. GDO0 output pin config. 0x06=0000 0110
    CC_SPIWriteReg(CCxxx0_PKTLEN,   0xFF); // Packet length. 0xFF=0b1111 1111
    CC_SPIWriteReg(CCxxx0_PKTCTRL1, 0x05); // Packet automation control. 0x05=0b0000 0101
    CC_SPIWriteReg(CCxxx0_PKTCTRL0, 0x05); // Packet automation control. 0x05=0b0000 0101
    CC_SPIWriteReg(CCxxx0_ADDR,     0x01); // Device address. 0x01=0b0000 0001
    CC_SPIWriteReg(CCxxx0_CHANNR,   0x00); // Channel number. 0x00=0b0000 0000
    CC_SPIWriteReg(CCxxx0_FSCTRL1,  0x07); // Freq synthesizer control. 0x07=0b0000 0111
    CC_SPIWriteReg(CCxxx0_FSCTRL0,  0x00); // Freq synthesizer control. 0x00=0b0000 0000
    CC_SPIWriteReg(CCxxx0_FREQ2,    0x5D); // Freq control word, high byte 0x5D=0b0101 1101
    CC_SPIWriteReg(CCxxx0_FREQ1,    0x93); // Freq control word, mid byte. 0x93=0b1001 0011
    CC_SPIWriteReg(CCxxx0_FREQ0,    0xB1); // Freq control word, low byte. 0xB1=0b1011 0001
    CC_SPIWriteReg(CCxxx0_MDMCFG4,  0x2D); // Modem configuration. 0x2D=0b0010 1101
    CC_SPIWriteReg(CCxxx0_MDMCFG3,  0x3B); // Modem configuration. 0x3B=0b0011 1011
    CC_SPIWriteReg(CCxxx0_MDMCFG2,  0x73); // Modem configuration. 0x73=0b0111 0011
    CC_SPIWriteReg(CCxxx0_MDMCFG1,  0x22); // Modem configuration. 0x22=0b0010 0010
    CC_SPIWriteReg(CCxxx0_MDMCFG0,  0xF8); // Modem configuration. 0xF8=0b1111 1000
    CC_SPIWriteReg(CCxxx0_DEVIATN,  0x00); // Modem dev (when FSK mod en). 0x00=0b0000 0000
    CC_SPIWriteReg(CCxxx0_MCSM1 ,   0x3F); // MainRadio Cntrl State Machine. 0x3F=0b0011 1111
    CC_SPIWriteReg(CCxxx0_MCSM0 ,   0x18); // MainRadio Cntrl State Machine. 0x18=0b0001 1000
    CC_SPIWriteReg(CCxxx0_FOCCFG,   0x1D); // Freq Offset Compens. Config. 0x1D=0b0001 1101
    CC_SPIWriteReg(CCxxx0_BSCFG,    0x1C); // Bit synchronization config. 0x1C=0b0001 1100
    CC_SPIWriteReg(CCxxx0_AGCCTRL2, 0xC7); // AGC control. 0xC7=0b1100 0111
    CC_SPIWriteReg(CCxxx0_AGCCTRL1, 0x00); // AGC control. 0x00=0b0000 0000
    CC_SPIWriteReg(CCxxx0_AGCCTRL0, 0xB2); // AGC control. 0xB2=0b1011 0010
    CC_SPIWriteReg(CCxxx0_FREND1,   0xB6); // Front end RX configuration. 0xB6=0b1011 0110
    CC_SPIWriteReg(CCxxx0_FREND0,   0x10); // Front end RX configuration. 0x10=0b0001 0000
    CC_SPIWriteReg(CCxxx0_FSCAL3,   0xEA); // Frequency synthesizer cal. 0xEA=0b1110 1010
    CC_SPIWriteReg(CCxxx0_FSCAL2,   0x0A); // Frequency synthesizer cal. 0x0A=0b0000 1010
    CC_SPIWriteReg(CCxxx0_FSCAL1,   0x00); // Frequency synthesizer cal. 0x00=0b0000 0000
    CC_SPIWriteReg(CCxxx0_FSCAL0,   0x11); // Frequency synthesizer cal. 0x11=0b0001 0001
    CC_SPIWriteReg(CCxxx0_FSTEST,   0x59); // Frequency synthesizer cal. 0x59=0b0101 1001
    CC_SPIWriteReg(CCxxx0_TEST2,    0x88); // Various test settings. 0x88=0b1000 1000
    CC_SPIWriteReg(CCxxx0_TEST1,    0x31); // Various test settings. 0x31=0b0011 0001
    CC_SPIWriteReg(CCxxx0_TEST0,    0x0B); // Various test settings. 0x0B=0b0000 1011
}

void readRFSettings(void)
{
  unsigned char Dummy = 0;
  
  // Read register settings
  Dummy = CC_SPIReadReg(CCxxx0_IOCFG2);   // GDO2 output pin config.
  Dummy = CC_SPIReadReg(CCxxx0_IOCFG0);   // GDO0 output pin config.
  Dummy = CC_SPIReadReg(CCxxx0_PKTLEN);   // Packet length.
  Dummy = CC_SPIReadReg(CCxxx0_PKTCTRL1); // Packet automation control.
  Dummy = CC_SPIReadReg(CCxxx0_PKTCTRL0); // Packet automation control.
  Dummy = CC_SPIReadReg(CCxxx0_ADDR);     // Device address.
  Dummy = CC_SPIReadReg(CCxxx0_CHANNR);   // Channel number.
  Dummy = CC_SPIReadReg(CCxxx0_FSCTRL1);  // Freq synthesizer control.
  Dummy = CC_SPIReadReg(CCxxx0_FSCTRL0);  // Freq synthesizer control.
  Dummy = CC_SPIReadReg(CCxxx0_FREQ2);    // Freq control word, high byte
  Dummy = CC_SPIReadReg(CCxxx0_FREQ1);    // Freq control word, mid byte.
  Dummy = CC_SPIReadReg(CCxxx0_FREQ0);    // Freq control word, low byte.  Dummy = CC_SPIReadReg(CCxxx0_MDMCFG4); // Modem configuration.
  Dummy = CC_SPIReadReg(CCxxx0_MDMCFG3); // Modem configuration.
  Dummy = CC_SPIReadReg(CCxxx0_MDMCFG2); // Modem configuration.
  Dummy = CC_SPIReadReg(CCxxx0_MDMCFG1); // Modem configuration.
  Dummy = CC_SPIReadReg(CCxxx0_MDMCFG0); // Modem configuration.
  Dummy = CC_SPIReadReg(CCxxx0_DEVIATN); // Modem dev (when FSK mod en)
  Dummy = CC_SPIReadReg(CCxxx0_MCSM1); //MainRadio Cntrl State Machine
  Dummy = CC_SPIReadReg(CCxxx0_MCSM0); //MainRadio Cntrl State Machine
  Dummy = CC_SPIReadReg(CCxxx0_FOCCFG); // Freq Offset Compens. Config
  Dummy = CC_SPIReadReg(CCxxx0_BSCFG); //  Bit synchronization config.
  Dummy = CC_SPIReadReg(CCxxx0_AGCCTRL2); // AGC control.
  Dummy = CC_SPIReadReg(CCxxx0_AGCCTRL1); // AGC control.
  Dummy = CC_SPIReadReg(CCxxx0_AGCCTRL0); // AGC control.
  Dummy = CC_SPIReadReg(CCxxx0_FREND1); // Front end RX configuration.
  Dummy = CC_SPIReadReg(CCxxx0_FREND0); // Front end RX configuration.
  Dummy = CC_SPIReadReg(CCxxx0_FSCAL3); // Frequency synthesizer cal.
  Dummy = CC_SPIReadReg(CCxxx0_FSCAL2); // Frequency synthesizer cal.
  Dummy = CC_SPIReadReg(CCxxx0_FSCAL1); // Frequency synthesizer cal.
  Dummy = CC_SPIReadReg(CCxxx0_FSCAL0); // Frequency synthesizer cal.
  Dummy = CC_SPIReadReg(CCxxx0_FSTEST); // Frequency synthesizer cal.
  Dummy = CC_SPIReadReg(CCxxx0_TEST2); // Various test settings.
  Dummy = CC_SPIReadReg(CCxxx0_TEST1); // Various test settings.
  Dummy = CC_SPIReadReg(CCxxx0_TEST0);  // Various test settings.
}


// PATABLE (0 dBm output power)
extern char paTable[] = {0xFB}; //0xFB=0b1111 1011
extern char paTableLen = 1;


/*************************************************************************//**
 \fn void RFSendPacket(char *txBuffer, char size)
  
This function transmits a packet with length up to 63 bytes.  To use this 
function, GD00 must be configured to be asserted when sync word is sent and
de-asserted at the end of the packet, which is accomplished by setting the
IOCFG0 register to 0x06, per the CCxxxx datasheet.  GDO0 goes high at
packet start and returns low when complete.  The function polls GDO0 to
ensure packet completion before returning.

\param [in] char *txBuffer. Pointer to a buffer containing the data to be transmitted
\param [in] char size. The size of the txBuffer
\param [out] none
\return none
       
*****************************************************************************/
void RFSendPacket(uint8_t *txBuffer, uint8_t size)
{
  CC_SPIWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size); // Write TX data
  CC_SPIStrobe(CCxxx0_STX);           // Change state to TX, initiating
                                            // data transfer
  
  // Wait GDO0 to go hi -> sync TX'ed
//  while (!(CC_GDO0_PxIN & CC_GDO0_PIN));
//  while (CC_GDO0_PxIN&CC_GDO0_PIN);
  while(!(LPC_GPIO3->DATA&0x01));
  while((LPC_GPIO3->DATA&0x01));
  
                                            // Wait GDO0 to clear -> end of pkt
  //CC_GDO0_PxIFG &= ~CC_GDO0_PIN;      // After pkt TX, this flag is set.  Has to be cleared before existing.
  GPIOIntClear(PORT3,0);
//  MRFI_CLEAR_GDO0_INT_FLAG();
}

/**************************************************************************//**
\fn char RFReceivePacket(char *rxBuffer, char *length)
																			 
Receives a packet of variable length (first byte in the packet must be the
length byte).  The packet length should not exceed the RXFIFO size.  To use
this function, APPEND_STATUS in the PKTCTRL1 register must be enabled.  It
is assumed that the function is called after it is known that a packet has
been received; for example, in response to GDO0 going low when it is
configured to output packet reception status.

The RXBYTES register is first read to ensure there are bytes in the FIFO.
This is done because the GDO signal will go high even if the FIFO is flushed
due to address filtering, CRC filtering, or packet length filtering.

\param [in] char *rxBuffer. Pointer to the buffer where the incoming data should be stored
\param [in] char *length. Pointer to a variable containing the size of the buffer where the
incoming data should be stored. After this function returns, that
variable holds the packet length.
\param [out]
\return char. 0x80:  CRC OK, 0x00:  CRC NOT OK (or no pkt was put in the RXFIFO due to filtering)
******************************************************************************/
uint8_t RFReceivePacket(uint8_t *rxBuffer, uint8_t *length)
{
  uint8_t status[2];
  uint8_t pktLen;

  if ((CC_SPIReadStatus(CCxxx0_RXBYTES) & CCxxx0_NUM_RXBYTES))
  {
    pktLen = CC_SPIReadReg(CCxxx0_RXFIFO); // Read length byte

    if (pktLen <= *length)                  // If pktLen size <= rxBuffer
    {
      CC_SPIReadBurstReg(CCxxx0_RXFIFO, rxBuffer, pktLen); // Pull data
      *length = pktLen;                     // Return the actual size
      CC_SPIReadBurstReg(CCxxx0_RXFIFO, status, 2);
                                            // Read appended status bytes
      return (uint8_t)(status[CCxxx0_LQI_RX]&CCxxx0_CRC_OK);
    }                                       // Return CRC_OK bit
    else
    {
      *length = pktLen;                     // Return the large size
      CC_SPIStrobe(CCxxx0_SFRX);      // Flush RXFIFO
      return 0;                             // Error
    }
  }
  else
      return 0;                             // Error
}
