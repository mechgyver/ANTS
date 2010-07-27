/**
 \file ssp.h
 \brief Header file for NXP LPC13xx Family Microprocessors
 \version 1.0
 \date 6/1/10
 \author A.Wahab
	
 Description of registers:
 - SSP SR: Status Register.
	 - TFE: Transmit FIFO Empty, bit 0.  1 if TX FIFO is empty.
	 - TNF: Transmit FIFO Not Full, bit 1.  1 if TX FIFO is not full.
	 - RNE: Receive FIFO Not Empty, bit 2.  0 if RX FIFO is empty.
	 - RFF: Receive FIFO Full, bit 3. 1 if Receive FIFO is full.
	 - BSY: Busy, bit 4.  0 if SSP controller is idle, 1 if it is sending/receiving a frame and/or the TX FIFO is not empty.
	 - Bits 7:4 are reserved.
 - SSP IMSC: Interrupt Mask Set/Clear Register.
	 - RORIM: Receive Overrun, bit 0.
	 - RTIM: Receive Timeout, bit 1.
	 - RXIM: RX FIFO is at least half full, bit 2.
	 - TXIM: TX FIFO is at least half empty, bit 3.
	 - Bits 7:4 are reserved.
 - SSP RIS: Raw Interrupt Status Register
	 - RORRIS, bit 0.  1 if another frame was completely received while the RX FIFO was full.  The ARM spec implies that the preceding frame data is overwritten by the new frame data when this occurs.
	 - RTRIS, bit 1.  1 if the RX FIFO is not empty, and has not been read for a "timeout period".
	 - RXRIS, bit 2.  1 if the RX FIFO is at least half full.
	 - TXRIS, bit 3.  1 if the TX FIFO is at least half empty.
	 - Bits 7:4 are reserved.
 - SSP MIS: Masked Interrupt Status Register. ISR should read this register to determine cause(s) of interrupt.
	 - RORMIS, bit 0.  1 if another frame was completely received while the RX FIFO was full and RORIM interrupt is enabled.
	 - RTMIS, bit 1.  1 if RX FIFO is not empty, hasn't been read for a "timeout period", and RTIM interrupt is enabled.
	 - RXMIS, bit 2.  1 if the RX FIFO is at least half full and the RXIM interrupt is enabled.
	 - TXMIS, bit 3.  1 if the TX FIFO is at least half empty and the TXIM interrupt is enabled.
	 - Bits 7:4 are reserved.
 - SSP ICR: Interrupt Clear Register.  Software writes one or more one(s) to clear corresponing interrupt condition(s). The other two interrupt conditions can be cleared by writing or reading the appropriate FIFO, or disabled by clearing bits in IMSC.
	 - RORIC, bit 0. Writing a 1 to this bit clears the "frame was received when RX FIFO was full" interrupt.
	 - RTIC, bit 1.  Writing a 1 to this bit clears the "RX FIFO was not empty and has not been read for a timeout period" interrupt.
 - SSP CR0: Control Register 0. Selects the serial clock rate, bus type, and data size
	 - DSS: Data Size Select, bits 3:0.  This field controls the number of bits transferred in each frame.
	 - 0011: 4-bit transfer
	 - 0100: 5-bit transfer
	 - ...
	 - 1111: 15-bit transfer
	 - FRF: Frame Format, bits 5:4.  00 for SPI mode.
	 - CPOL: Clock Out Polarity, bit 6.  0 for idle low.
	 - CPHA: Clock Out Phase, bit 7.  0 for capture data on first clock transition of frame
	 - SCR: Serial Clock Rate, bits 15:8.  Number of prescaler-output clocks per bit on the bus, minus one.
 - SSP CR1: Control Register 1. Selects master/slave and other modes.
	 - LBM: Loop Back Mode, bit 0.  0 for normal operation.
	 - SSE: SSP Enable, bit 1.  0 for disable.
	 - MS: Master/Slave Mode, bit 2.  0 for Master Mode (only set when SSE bit is 0).
	 - SOD: Slave Output Disable, bit 3.  1 to block SSP controller from driving MISO line (only relevant in slave mode).
	 - Bits 7:4 are reserved.
 */

#ifndef __SSP_H__
#define __SSP_H__

#include "stdint.h"
/*
 */
#define SSP_BUFSIZE		16	//16/**< SPI read and write buffer size */

#define FIFOSIZE		8       //8

#define DELAY_COUNT		10
#define MAX_TIMEOUT		0xFF

#define SSP0_SEL		(1 << 2) /**< Port0.2 is the SSP select pin */
	
/* SSP Status Register */
#define SSPSR_TFE		(1 << 0)
#define SSPSR_TNF		(1 << 1) 
#define SSPSR_RNE		(1 << 2)
#define SSPSR_RFF		(1 << 3) 
#define SSPSR_BSY		(1 << 4)

/* SSP Control Register 0 */
#define SSPCR0_DSS		(1 << 0) //original:(1<<0) 0b0000 0000 0000 0001 
#define SSPCR0_FRF		(1 << 4) //original:(1<<4) 0b0000 0000 0001 0000
#define SSPCR0_SPO		(1 << 5) //original:(1<<6) 0b0000 0000 0100 0000
#define SSPCR0_SPH		(1 << 7) //original:(1<<7) 0b0000 0000 1000 0000
#define SSPCR0_SCR		(1 << 8) //original:(1<<8) 0b0000 0001 0000 0000

/* SSP Control Register 1 */
#define SSPCR1_LBM		(1 << 0)
#define SSPCR1_SSE		(1 << 1)
#define SSPCR1_MS		(1 << 2)
#define SSPCR1_SOD		(1 << 3)

/* SSP Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM	(1 << 0)
#define SSPIMSC_RTIM	(1 << 1)
#define SSPIMSC_RXIM	(1 << 2)
#define SSPIMSC_TXIM	(1 << 3)

/* SSP0 Interrupt Status register */
#define SSPRIS_RORRIS	(1 << 0)
#define SSPRIS_RTRIS	(1 << 1)
#define SSPRIS_RXRIS	(1 << 2)
#define SSPRIS_TXRIS	(1 << 3)

/* SSP0 Masked Interrupt register */
#define SSPMIS_RORMIS	(1 << 0)
#define SSPMIS_RTMIS	(1 << 1)
#define SSPMIS_RXMIS	(1 << 2)
#define SSPMIS_TXMIS	(1 << 3)

/* SSP0 Interrupt clear register */
#define SSPICR_RORIC	(1 << 0)
#define SSPICR_RTIC		(1 << 1)

/* If RX_INTERRUPT is enabled, the SSP RX will be handled in the ISR
SSPReceive() will not be needed. */
extern void SSP0_IRQHandler(void);
extern void SSPInit( void );
extern void SSPSend( uint8_t *Buf, uint32_t Length );
extern void SSPReceive( uint8_t *buf, uint32_t Length );

#endif  /* __SSP_H__ */
/*****************************************************************************
*								End Of File
******************************************************************************/

