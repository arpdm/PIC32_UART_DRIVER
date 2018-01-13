#ifndef __UARTDRIVER_H
#define	__UARTDRIVER_H

#include "System.h"

//===================================ABOUT===================================

/*
 *  @Module_Name:   UARTDriver
 *  @AUTHOR:        Arpi Derm
 *  @RELEASE_DATE:  June 29, 2017, 4:13 PM
 *  @Description:   This module handles all the UART TX and RX using Interrupts
 */

//==============================REVISION HISTORY=============================

/*  @DATE: July 14,2017
 *  @DATE: (RP) Change Description...
 * 
 */

//==============================PUBLIC INTERFACE=============================

/***************************************************************************
 * Function :  UART1InitializeDriver
 * Input    :  none
 * Output   :  none
 * Comment  :  Initialize the UART
 *             We want to set port pins in safe state first Set Pin Direction (this 
 *             may seem Redundant, but we do this to keep the pins in a safe state
 ***************************************************************************/

void UART1InitializeDriver(void);

/***************************************************************************
 * Function :  U1ReadSingleByte
 * Input    :  none
 * Output   :  char
 * Comment  :  Read Individual Character from buffer
 ***************************************************************************/

unsigned char U1ReadSingleByte(void);

/***************************************************************************
 * Function :  U1GetReadCount
 * Input    :  none
 * Output   :  uint16_t - Number of bytes available to read
 * Comment  :  Returns number of available bytes in RX Buffer to read
 ***************************************************************************/

uint16_t U1GetReadCount(void);

/***************************************************************************
 * Function :  U1WriteSingleByte
 * Input    :  input byte - one single byte
 * Output   :  none
 * Comment  :  Send Single byte 
 ***************************************************************************/

bool U1WriteSingleByte(const uint8_t inputByte);

/***************************************************************************
 * Function :  U1WriteManyBytes
 * Input    :  const uint8_t *buffer - pointer to input string
 *             const uint16_t length - number of bytes to Tx
 * Output   :  uint8_t number of bytes actually written (regardless of requested)
 * Comment  :  Send Single byte 
 ***************************************************************************/

uint8_t U1WriteManyBytes(const uint8_t *buffer, const uint8_t length);



#endif // __UARTDRIVER_H