/*
	* DSerial Bootloader
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

	Change Log
	==========
	v1.2
	- added SELECT_VERSION to check hardware version of DSerial
	- MOSI line is now push-pull instead of open-collector

	v1.0
	- initial code
	- supports SELECT_INTERRUPT, SELECT_UART_BUFFER, SELECT_CHECK,
	  SELECT_BOOT, SELECT_FLASH_ERASE and SELECT_FLASH commands

*/

//-----------------------------------------------------------------------------
// Compiler options: -c
// Linker options: --code-size 62464 --xram-size 4096 --iram-size 256
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f340.h>
#include "../common.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

#define FLBWE 		0x01 		/* PFE0CN */
#define PFEN 		0x20 		/* PFE0CN */

//-----------------------------------------------------------------------------
// Global varibles
//-----------------------------------------------------------------------------

volatile bit BootFirmware = 0;

volatile unsigned char SpiCommand = 0;
volatile unsigned char SpiSize = 0;
volatile unsigned char SpiSendState = 0;
volatile unsigned char SpiInterruptStatus = 0;

volatile xdata t_buffer UartSendBuffer;
volatile xdata t_circular UartRecvBuffer;
volatile t_fw_buffer FirmwareBuffer;

//-----------------------------------------------------------------------------
// Declarations
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Init Routines
//-----------------------------------------------------------------------------

void clockInit() {
	//select SYSCLK of 24MHz
	int i = 0;
	CLKMUL    = 0x80;
	for (i = 0; i < 20; i++);			// Wait 5us for initialization
	CLKMUL    |= 0xC0;
	while ((CLKMUL & 0x20) == 0);
	CLKSEL    = 0x02;
	OSCICN    = 0x83;
}

/*
void clockInit() {
	//select SYSCLK of 48MHz
    int i = 0;
    FLSCL     = 0x90;
    CLKMUL    = 0x80;
    for (i = 0; i < 20; i++);			// Wait 5us for initialization
    CLKMUL    |= 0xC0;
    while ((CLKMUL & 0x20) == 0);
    CLKSEL    = 0x03;
    OSCICN    = 0x83;
	PFE0CN    |= PFEN;					// Enable prefetch
}
*/


void portInit(void) {
	P0MDOUT   = 0x02;					// MISO
	P1MDOUT   = 0x03;					// LEDs
	P2MDIN    = 0xFC;					// Port 2 pin 0 and 1 set as analog input
	P3MDOUT   = 0x01;					// DS_IRQ line
	P3MDIN    = 0x01;					// DS_IRQ line , digital pin
	P2SKIP    = 0x03;					// Port 2 pin 0 and 1 and skipped by crossbar
	XBR0      = 0x03;					// Crossbar setup
	XBR1      = 0x40;					// Enable Crossbar
}

void spiInit(void) {
	SPI0CFG   = 0x30;					// rising edge, idle high
	SPI0CN    = 0x05;
	ESPI0     = 1;						// SPI0 interrupt enable
	PSPI0	  = 1;						// SPI0 interrupt priority high
}

void uartInit() {
	SCON0     = 0x10;					// mode 1

	Uart0Tx = 1;						// TX and RX high
	Uart0Rx = 1;

	// UART0 uses Timer1 for baud rate generation

	//9600bps
    //TMOD      = 0x20;
    //TH1       = 0x98;
	//TL1 	  = 0x98;
    //TMR2CN    = 0x04;

	//115200bps
    TMOD      = 0x20;
    CKCON     = 0x08;
    TH1       = 0x98;					// 115200 bps

	TR1 	  = 1;						// run timer

	REN0	  = 1;
	PS0		  = 1;						// low priority
	ES0		  = 1;						// enable interrupt

	UartRecvBuffer.read = 0;
	UartRecvBuffer.write = 0;

	UartEn	  = 0;						// enable UART
}

//-----------------------------------------------------------------------------
// Helper Routines
//-----------------------------------------------------------------------------

void flashEraseBlock(unsigned int dest) {
	xdata unsigned char *flash = (xdata unsigned char *) dest;

	EA = 0;				// disable interrupts
	FLKEY = 0xA5;
	FLKEY = 0xF1;

	PSCTL |= PSEE;
	PSCTL |= PSWE;		// Program Store Write Enable

	*flash = 0;

	PSCTL &= ~PSWE;
	PSCTL &= ~PSEE;
	EA = 1;				// enable interrupts
}

void flashWrite(unsigned int dest, unsigned char * src, unsigned char size) {
	xdata unsigned char *flash = (xdata unsigned char *) dest;
	unsigned int i;

	EA = 0;				// disable interrupts

	PFE0CN &= ~FLBWE;	// select single byte mode
	PSCTL |= PSWE; 		// Program Store Write Enable
	PSCTL &= ~PSEE;

	for(i = 0; i < size; i++) {
		data unsigned char c = *src++;

		FLKEY = 0xA5;
		FLKEY = 0xF1;

		*flash++ = c;
	}

	PSCTL &= ~PSWE;

	EA = 1;				// enable interrupts
}

void flashRead(unsigned char * dest, unsigned int src, unsigned char size) {
	code unsigned char *flash = (code unsigned char *) src;
	unsigned int i;

	for(i = 0; i < size; i++) {
		*dest++ = *flash++;
	}

}

void bootFirmware() {
	_asm
		clr	a
		mov ie, a
		mov ip, a
		mov	psw, a
		mov dpl, a
		mov dph, a
		mov r7, a
		mov r6, a
		mov r5, a
		mov r4, a
		mov r3, a
		mov r2, a
		mov r1, a
		mov r0, a
		mov sp, #7

		ljmp #FIRMWARE_OFFSET
	_endasm;
}

//-----------------------------------------------------------------------------
// Interrupt Routines
//-----------------------------------------------------------------------------

void spiInterrupt(void) interrupt using 2 {
	if (SPIF) {							// transfer interrupt

		// receive
		unsigned char c = SPI0DAT;
		SPIF = 0;						// ack interrupt

		//Led0 = 0;
		//Led1 = 1;

		switch (SpiCommand) {
		case 0:
			if (c != 0) {
				SpiCommand = c;			// command byte
				SpiSendState = 1;
				switch (SpiCommand) {
				case (SELECT_READ | SELECT_INTERRUPT):
					SPI0DAT = 1; 		// send size
					break;
				case (SELECT_READ | SELECT_UART0_BUFFER):
					SpiSize = UartRecvBuffer.write;
					// send size (circular buffer)
					if (SpiSize > UartRecvBuffer.read) {
						SPI0DAT = SpiSize - UartRecvBuffer.read;
					} else {
						SPI0DAT = MAX_DATA_SIZE - UartRecvBuffer.read + SpiSize;
					}
					break;
				case (SELECT_READ | SELECT_CHECK):
					SPI0DAT = 1; 		// send size
					break;
				case (SELECT_READ | SELECT_VERSION):
					SPI0DAT = 1; 		// send size
					break;
				case (SELECT_BOOT):
					BootFirmware = 1;
					// reset SPI command receiver
					SpiCommand = 0;
					break;
				}
			}
			break;
		case (SELECT_READ | SELECT_INTERRUPT):
			// send status to DS
			SPI0DAT = SpiInterruptStatus;
			// de-assert irq line
			DsIrq = 0;
			// reset SPI command receiver
			SpiCommand = 0;
			break;
		case (SELECT_READ | SELECT_UART0_BUFFER):
			// send next UART byte
			SPI0DAT = UartRecvBuffer.d[UartRecvBuffer.read];
			// advance index in circular buffer
			UartRecvBuffer.read = (UartRecvBuffer.read + 1) & 0x1F;
			if (UartRecvBuffer.read == SpiSize) {
				// reset SPI command receiver
				SpiCommand = 0;
			}
			break;
		case (SELECT_WRITE | SELECT_INTERRUPT):
			if (SpiSendState == 1) {	// size byte
				SpiSize = c;
				SpiSendState++;
			} else {					// data byte
				// clear bits specified
				SpiInterruptStatus &= ~c;
				// reset SPI command receiver
				SpiCommand = 0;
			}
			break;
		case (SELECT_WRITE | SELECT_UART0_BUFFER):
			if (SpiSendState == 1) {	// size byte
				UartSendBuffer.size = c;
				UartSendBuffer.curr = 0;
				SpiSendState++;
			} else {					// data byte
				UartSendBuffer.d[SpiSendState-2] = c;
				SpiSendState++;
				if (SpiSendState == UartSendBuffer.size + 2) {
					// start sending data to UART
					SBUF0 = UartSendBuffer.d[0];
					// reset SPI command receiver;
					SpiCommand = 0;
				}
			}
			break;
		case (SELECT_WRITE | SELECT_FLASH_ERASE):
			switch (SpiSendState) {
			case 1:	// size byte
				SpiSize = c;
				break;
			case 2:
				FirmwareBuffer.dest = ((unsigned int) c) << 8; // big-endian
				break;
			default:
				FirmwareBuffer.dest |= (unsigned int) c;
				if (FirmwareBuffer.dest >= FIRMWARE_OFFSET) {  // protect bootloader
					flashEraseBlock(FirmwareBuffer.dest);
				}
				// reset SPI command receiver;
				SpiCommand = 0;
				// generate interrupt
				SpiInterruptStatus |= INTERRUPT_BOOTLOADER;
				DsIrq = 1; // DS interrupt
				break;
			}
			SpiSendState++;
			break;
		case (SELECT_WRITE | SELECT_FLASH):
			switch (SpiSendState) {
			case 1:	// size byte
				FirmwareBuffer.size = c-2;
				break;
			case 2:
				FirmwareBuffer.dest = ((unsigned int) c) << 8; // big-endian
				break;
			case 3:
				FirmwareBuffer.dest |= (unsigned int) c;
				break;
			default:
				FirmwareBuffer.d[SpiSendState-4] = c;
				if (SpiSendState-3 == FirmwareBuffer.size) {
					if (FirmwareBuffer.dest >= FIRMWARE_OFFSET) {  // protect bootloader
						flashWrite(FirmwareBuffer.dest, FirmwareBuffer.d, FirmwareBuffer.size);
					}
					// reset SPI command receiver;
					SpiCommand = 0;
					// generate interrupt
					SpiInterruptStatus |= INTERRUPT_BOOTLOADER;
					DsIrq = 1; // DS interrupt
				}
				break;
			}
			SpiSendState++;
			break;
		case (SELECT_READ | SELECT_FLASH): // non-standard command!!!
			switch (SpiSendState) {
			case 1:	// size byte
				FirmwareBuffer.size = c;
				break;
			case 2:
				FirmwareBuffer.dest = ((unsigned int) c) << 8; // big-endian
				break;
			default:
				FirmwareBuffer.dest |= (unsigned int) c;

				SPI0DAT = ((code unsigned char *) FirmwareBuffer.dest)[SpiSendState-3];
				if (SpiSendState-2 == FirmwareBuffer.size) {
					// reset SPI command receiver;
					SpiCommand = 0;
				}
				break;
			}
			SpiSendState++;
			break;
		case (SELECT_READ | SELECT_CHECK):	// special command
			// send DSerial signature byte (hopefully no EEPROM has this one...)
			SPI0DAT = 0xAB;
			// reset SPI command receiver;
			SpiCommand = 0;
			break;
		case (SELECT_READ | SELECT_VERSION):
			// send DSerial version
			SPI0DAT = 0x02; // Version 2 = DSerial Edge
			// reset SPI command receiver;
			SpiCommand = 0;
			break;
		default:
			// reset SPI command receiver;
			SpiCommand = 0;
		}

	} // SPIF

	if (WCOL) {							// write collision interrupt

		WCOL = 0;
	}

	if (MODF) {							// mode fault interrupt

		MODF = 0;
	}

	if (RXOVRN) {						// receive overrun interrupt

		RXOVRN = 0;
	}
}




void uartInterrupt(void) interrupt {
	if (RI0) {								// receive interrupt
		RI0 = 0;
		//Led0 = 1;
		//SBUF0 = SBUF0;	//echo


		UartRecvBuffer.d[UartRecvBuffer.write] = SBUF0;
		UartRecvBuffer.write = (UartRecvBuffer.write + 1) & 0x1F;
		// generate interrupt
		SpiInterruptStatus |= INTERRUPT_UART0_RX;
		DsIrq = 1; // DS interrupt

	}

	if (TI0) {								// transmit interrupt
		TI0 = 0;							// ack interrupt

		UartSendBuffer.curr++;
		if (UartSendBuffer.curr == UartSendBuffer.size) {
			// stop sending
			UartSendBuffer.size = 0;
			// generate interrupt
			SpiInterruptStatus |= INTERRUPT_UART0_TX;
			DsIrq = 1; // DS interrupt
		} else {
			// send next byte
			SBUF0 = UartSendBuffer.d[UartSendBuffer.curr];
		}
	}

}

//-----------------------------------------------------------------------------
// Interrupt Redirection
//-----------------------------------------------------------------------------

#include "interrupt.h"

//-----------------------------------------------------------------------------
// Main Routine
//-----------------------------------------------------------------------------

void main() {
	PCA0MD &= ~0x40;					// Disable Watchdog timer
	F1 = 1;								// Select bootloader interrupts

	portInit();
	clockInit();
	spiInit();
	uartInit();

	EA = 1; 							// Global interrupt enable

	Led0 = 1;
	Led1 = 0;
	DsIrq = 0;

	BootFirmware = 0;

	// we don't want to boot to firmware from an interrupt
	// so we do it here, if it's requested by DS
	for(;;) {
		if (BootFirmware) {
			bootFirmware();
		}
	}
}
