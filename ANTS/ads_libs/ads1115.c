/***************************************************************************//**
\file ads1115.c
\brief Functions to configure and communicate with TI ADS1115 A/D converter.
*******************************************************************************/


#include "ads1115.h"

//******************************************************************************
void ads_init(void)
{
/***************************************************************************//**
Sending the following bytes will configure the ADS for continuous conversion mode
-First byte sent is 7-bit I2C address followed by a write bit: ((ADS_ADDRESS << 1) | (ADS_WR_BIT))
-Second byte sent points to Config Register address: (ADS_CONFIG)
-Third byte is MSB of data to be written to Config Register
-Fourth byte is LSB of data to be written to Config Register

Sending the following bytes will ...see datasheet example
*******************************************************************************/
}