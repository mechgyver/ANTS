/***************************************************************************//**
\file ads1115.h
\brief Library of registers and commands for the TI ADS1115 A/D converter.
*******************************************************************************/

#define	ADS_ADDRESS	0x48	// default I2C slave address (7-bits) when ADDR pin is conected to GND, 0b1001000

#define ADS_RW_BIT	0x1		// append to address as LS bit; 1 indicates read

#define ADS_CONVER	0x00	// Conversion Register address
#define ADS_CONFIG	0x01	// Config Register address

/***************************************************************************//**
Lo_thresh and Hi_thresh Registers
- Upper and lower threshold values (16-bits each) used by the comparator
- Twos Complement format
- Be sure to readjust values whenever PGA settings are modified
- Lo_thresh default = 0x8000
- Hi_thresh default = 0x7FFF
*******************************************************************************/
#define ADS_LOTHR	0x02	// Lo_thresh Register address
#define ADS_HITHR	0x03	// Hi_thresh Register address

/***************************************************************************//**
Pointer Register
- To access a register, the master must first write an appropriate value to the Pointer Register
- [slave address, 7-bits][low R/W bit][successful slave acknowledgment][now write byte to pointer register]
- No need to continually send Point Register bytes if repeatedly reading from the same register
- Every write operation requires the Pointer Register to be written
*******************************************************************************/
#define ADS_POINTR	0x00	// 6 MS bits of Pointer Register Byte (write-only)

/***************************************************************************//**
Config Register bits
- Bit[15] OS, operational status/single-shot conversion start
	- Determines operational status of device
	- Bit can only be written to when in power-down mode
	- For a write status: 0=no effect, 1=begin a single conversion (when in power-down mode)
	- For a read status: 0=device is currently performing a conversion, 1=devices is not currently performing a conversion
- Bits[14:12] MUX[2:0], input mux configuration
	- 000: AINp=AIN0, AINn=AIN1 (default)
	- 001: AINp=AIN0, AINn=AIN3
	- 010: AINp=AIN1, AINn=AIN3
	- 011: AINp=AIN2, AINn=AIN3
	- 100: AINp=AIN0, AINn=GND
	- 101: AINp=AIN1, AINn=GND
	- 110: AINp=AIN2, AINn=GND
	- 111: AINp=AIN3, AINn=GND
- Bits[11:9] PGA[2:0], programmable gain amplifier configuration
	- 000: FS = +/-6.144 V
	- 001: FS = +/-4.096 V
	- 010: FS = +/-2.048 V 
	- 011: FS = +/-1.024 V 
	- 100: FS = +/-0.512 V
	- 101: FS = +/-0.256 V
	- 110: FS = +/-0.256 V
	- 111: FS = +/-0.256 V
- Bit[8] MODE, controls device operating mode
	- 0: Continuous conversion mode
	- 1: Power-down single-shot mode (default)
- Bits[7:5] DR, control data rate setting
	- 000: 8 SPS
	- 001: 16 SPS
	- 010: 32 SPS
	- 011: 64 SPS
	- 100: 128 SPS (default)
	- 101: 250 SPS
	- 110: 475 SPS
	- 111: 860 SPS
- Bit[4] COMP_MODE, controls the comparator mode of operation
	- 0: Traditional comparator with hysteresis (default)
	- 1: Window comparator
- Bit[3] COMP_POL, controls the polarity of the ALERT/RDY pin
	- 0: Active low
	- 1: Active high
- Bit[2] COMP_LAT, controls latchinge behaviour of ALERT/RDY pin when conversions are within the margin of the upper and lower threshold values.
	- 0: Non-latching (default), ALERT/RDY pin does not latch when asserted
	- 1: ALERT/RDY pin latches and remains this way until conversion data is read by the master
- Bits[1:0] COMP_QUE, comparator queue and disable (two functons)
	- 00: Assert ALERT/RDY pin after one conversion
	- 01: Assert ALERT/RDY pin after two conversions
	- 10: Assert ALERT/RDY pin after four conversions
	- 11: Disable comparator (default) and put ALERT/RDY pin into a high state 
*******************************************************************************/
#define ADS_OS			(1<<15)
#define ADS_MUX2		(1<<14)
#define ADS_MUX1		(1<<13)
#define ADS_MUX0		(1<<12)
#define	ADS_PGA2		(1<<11)
#define	ADS_PGA1		(1<<10)
#define	ADS_PGA0		(1<<9)
#define	ADS_MODE		(1<<8)
#define	ADS_DR2			(1<<7)
#define	ADS_DR1			(1<<6)
#define	ADS_DR0			(1<<5)
#define	ADS_COMP_MODE	(1<<4)
#define	ADS_COMP_POL	(1<<3)
#define	ADS_COMP_LAT	(1<<2)
#define	ADS_COMP_QUE1	(1<<1)
#define	ADS_COMP_QUE0	(1<<0)

/***************************************************************************//**
Function Prototypes
*******************************************************************************/
void ads_init(void);