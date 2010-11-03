/*
	* DSerial Firmware/Bootloader
	*
	* Copyright (c) 2008, Alexei Karpenko
	* All rights reserved.
	* Redistribution and use in source and binary forms, with or without
	* modification, are permitted provided that the following conditions are met:
	*
	*     * Redistributions of source code must retain the above copyright
	*       notice, this list of conditions and the following disclaimer.
	*     * Redistributions in binary form must reproduce the above copyright
	*       notice, this list of conditions and the following disclaimer in the
	*       documentation and/or other materials provided with the distribution.
	*     * Neither the name of the <organization> nor the
	*       names of its contributors may be used to endorse or promote products
	*       derived from this software without specific prior written permission.
	*
	* THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
	* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	* DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
	* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <c8051f340.h>

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

#define FIRMWARE_OFFSET		0x0800
#define MAX_DATA_SIZE		32

#define TRUE				1
#define FALSE				0

#define Led0				P1_0
#define Led1				P1_1
#define DsIrq				P3_0

#define Uart0Tx				P0_4
#define Uart0Rx				P0_5
#define UartEn				P0_6
#define Uart1Tx				P2_6
#define Uart1Rx				P2_7


// Selectors

#define SELECT_READ				0x00
#define SELECT_WRITE			0x80
#define SELECT_INTERRUPT		0x01
#define SELECT_CHECK			0x9F

#define SELECT_FLASH			0x02
#define SELECT_FLASH_ERASE		0x03
#define SELECT_BOOT				0x04
#define SELECT_REGISTER			0x05
#define SELECT_ENABLE			0x06
#define SELECT_VERSION			0x07

#define SELECT_UART0_BUFFER		0x11
#define SELECT_UART1_BUFFER		0x12

#define SELECT_ADC				0x20
#define SELECT_ADC_SEQUENCE		0x21

#define SELECT_SERVO			0x30


// Enable flags
#define ENABLE_RS232			0x01
#define ENABLE_SERVO			0x02


// Interrupt flags

#define INTERRUPT_BOOTLOADER	0x01
#define INTERRUPT_UART0_TX		0x02
#define INTERRUPT_UART0_RX		0x04
#define INTERRUPT_UART1_TX		0x08
#define INTERRUPT_UART1_RX		0x10


// Type defines

typedef struct {
	unsigned char size;
	unsigned char curr;
	unsigned char d[32];
} t_buffer;

typedef struct {
	unsigned char read;
	unsigned char write;
	unsigned char d[32];
} t_circular;

typedef struct {
	unsigned char size;
	unsigned int  dest;
	unsigned char d[32];
} t_fw_buffer;

