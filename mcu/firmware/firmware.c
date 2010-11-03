/*
	* DSerial Firmware
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
	
	v1.0
	- initial code
	- supports SELECT_INTERRUPT, SELECT_UART_BUFFER, SELECT_CHECK,
	  SELECT_BOOT_FIRMWARE, SELECT_FLASH_ERASE and SELECT_FLASH commands

*/

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f340.h>
#include "../common.h"
#include "registers.h"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

#define FLBWE 		0x01 		/* PFE0CN */
#define PFEN 		0x20 		/* PFE0CN */

#define RI1 		0x01		/* SCON1 */
#define TI1			0x02		/* SCON1 */

#define ES1			0x02		/* EIE2 */

//-----------------------------------------------------------------------------
// Global varibles
//-----------------------------------------------------------------------------

volatile bit BootBootloader = 0;

volatile unsigned char SpiCommand = 0;
volatile unsigned char SpiSize = 0;
volatile unsigned char SpiSendState = 0;
volatile unsigned char SpiInterruptStatus = 0;

volatile xdata t_buffer UartSendBuffer[2];
volatile xdata t_circular UartRecvBuffer[2];
volatile t_fw_buffer FirmwareBuffer;

volatile xdata unsigned short AdcBuffer[16];
volatile xdata unsigned char AdcSequence[16];
volatile unsigned char AdcIndex = 0;
volatile unsigned char AdcReadIndex = 0;

volatile xdata unsigned char Servo[5];

//-----------------------------------------------------------------------------
// Declarations
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Init Routines
//-----------------------------------------------------------------------------

void clockInit24() {
	int i = 0;

	// set clock to default state
	CLKSEL = 0x10;
	CLKMUL = 0x00;
	OSCICN = 0x83;
	PFE0CN &= ~PFEN;					// disable prefetch

	// select SYSCLK of 24MHz
	CLKMUL    = 0x80;
	for (i = 0; i < 20; i++);			// Wait 5us for initialization
	CLKMUL    |= 0xC0;
	while ((CLKMUL & 0x20) == 0);
	CLKSEL    = 0x02;
	OSCICN    = 0x83;
}

void clockInit48() {
	int i = 0;

	// set clock to default state
	CLKSEL = 0x10;
	CLKMUL = 0x00;
	OSCICN = 0x83;

	// select SYSCLK of 48MHz
    FLSCL     = 0x90;
    CLKMUL    = 0x80;
    for (i = 0; i < 20; i++);			// Wait 5us for initialization
    CLKMUL    |= 0xC0;
    while ((CLKMUL & 0x20) == 0);
    CLKSEL    = 0x03;
    OSCICN    = 0x83;
	PFE0CN    |= PFEN;					// Enable prefetch
}


void portInit(void) {  
    // P0.0  -  SCK  (SPI0), Open-Drain, Digital
    // P0.1  -  MISO (SPI0), Push-Pull,  Digital
    // P0.2  -  MOSI (SPI0), Open-Drain, Digital
    // P0.3  -  NSS  (SPI0), Open-Drain, Digital
    // P0.4  -  TX0 (UART0), Open-Drain, Digital
    // P0.5  -  RX0 (UART0), Open-Drain, Digital
    // P0.6  -  Skipped,     Open-Drain, Digital
    // P0.7  -  Skipped,     Open-Drain, Digital

    // P1.0  -  Skipped,     Push-Pull,  Digital
    // P1.1  -  Skipped,     Push-Pull,  Digital
    // P1.2  -  Skipped,     Open-Drain, Digital
    // P1.3  -  Skipped,     Open-Drain, Digital
    // P1.4  -  Skipped,     Open-Drain, Digital
    // P1.5  -  Skipped,     Open-Drain, Digital
    // P1.6  -  Skipped,     Open-Drain, Digital
    // P1.7  -  Skipped,     Open-Drain, Digital

    // P2.0  -  Skipped,     Open-Drain, Digital
    // P2.1  -  Skipped,     Open-Drain, Digital
    // P2.2  -  Skipped,     Open-Drain, Digital
    // P2.3  -  Skipped,     Open-Drain, Digital
    // P2.4  -  Skipped,     Open-Drain, Digital
    // P2.5  -  Skipped,     Open-Drain, Digital
    // P2.6  -  TX1 (UART1), Open-Drain, Digital
    // P2.7  -  RX1 (UART1), Open-Drain, Digital

    // P3.0  -  Unassigned,  Push-Pull,  Digital

    P0MDOUT   = 0x02;
    P1MDOUT   = 0x03;
    P3MDOUT   = 0x01;
    P0SKIP    = 0xC0;
    P1SKIP    = 0xFF;
    P2SKIP    = 0x3F;
    XBR0      = 0x03;
    XBR1      = 0x40;
    XBR2      = 0x01;
}

void spiInit(void) {
	SPI0CFG   = 0x30;					// rising edge, idle high
	SPI0CN    = 0x05;
	ESPI0     = 1;						// SPI0 interrupt enable
	PSPI0	  = 1;						// SPI0 interrupt priority high
}

void uart0Init() {
	SCON0     = 0x10;					// mode 1

	Uart0Tx = 1;						// TX and RX high
	Uart0Rx = 1;

	// UART0 uses Timer1 for baud rate generation

	// 115200 bps
    TMOD      = 0x20;
    CKCON     = 0x08;
    TH1       = 0x30;
	
	TR1 	  = 1;						// run timer

	REN0	  = 1;
	PS0		  = 0;						// low priority
	ES0		  = 1;						// enable interrupt

	UartRecvBuffer[0].read = 0;
	UartRecvBuffer[0].write = 0;

	UartEn	  = 0;						// enable UART
}

void uart1Init() {
	Uart1Tx = 1;						// TX and RX high
	Uart1Rx = 1;

    SCON1     = 0x10;
    SBCON1    = 0x43;

	// 9600 bps
    SBRLL1    = 0x3C;
    SBRLH1    = 0xF6;

	UartRecvBuffer[1].read = 0;
	UartRecvBuffer[1].write = 0;

	// enable UART1 interrupt
	EIE2 |= ES1;
}

void adcInit() {
	REF0CN	  = 0x0E;					// Enable voltage reference VREF
	AMX0N      = 0x1F;					// Single ended mode(negative input = gnd)

	//ADC0CF     = 0xFC;					// SAR Period 0x1F, left-adjusted output

	ADC0CN     = 0xC2;					// Continuous conversion on timer 2 overflow
										// with low power tracking mode on

	EIE1   |= 0x08;						// Enable ADC conversion complete interrupt
}

void timer2Init() {
	TMR2CN  = 0x00;						// Stop Timer2; Clear TF2;

	CKCON  &= ~0xF0;					// Timer2 clocked based on T2XCLK;
	TMR2RL  = 0x7B80;					// Initialize reload value
	TMR2    = 0xFFFF;					// Set to reload immediately

	//ET2     = 1;						// Enable Timer2 interrupts
	TR2     = 1;						// Start Timer2
}

void pcaInit() {
	PCA0CP0 = 0xF82F; // servo left 	+0x3E8
	PCA0CP1 = 0xF447; // center servo
	PCA0CP2 = 0xF05F; // servo right	-0x3E8


    //PCA0CN    = 0x40;
    PCA0MD    &= ~0x40;
    PCA0MD    = 0x00;
    PCA0CPM0  = 0xC2;
    PCA0CPM1  = 0xC2;
    PCA0CPM2  = 0xC2;
    PCA0CPM3  = 0xC2;
    PCA0CPM4  = 0xC2;

	PCA0 = 0x63BF; 						// 20 ms cycle, scope shows 32ms... 

	PCA0CN    = 0x40;					// enable PCA0

	//EIE1     |= 0x10;					// enable ADC interrupts
}

//-----------------------------------------------------------------------------
// Helper Routines
//-----------------------------------------------------------------------------

void flashEraseBlock(unsigned int dest) {
	xdata unsigned char *flash = (xdata unsigned char *) dest;

	EA = 0;			// disable interrupts
	FLKEY = 0xA5;
	FLKEY = 0xF1;

	PSCTL |= PSEE;
	PSCTL |= PSWE;	// Program Store Write Enable

	*flash = 0;

	PSCTL &= ~PSWE;
	PSCTL &= ~PSEE;
	EA = 1;			// enable interrupts
}

void flashWrite(unsigned int dest, unsigned char * src, unsigned char size) {
	xdata unsigned char *flash = (xdata unsigned char *) dest;
	unsigned int i;

	EA = 0;			// disable interrupts

	for(i = 0; i < size; i++) {
		data unsigned char c = *src++;

		FLKEY = 0xA5;
		FLKEY = 0xF1;

		PSCTL |= PSWE; // Program Store Write Enable
		PSCTL &= ~PSEE;

		*flash++ = c;

		PSCTL &= ~PSWE;
	}

	EA = 1;			// enable interrupts
}

void flashRead(unsigned char * dest, unsigned int src, unsigned char size) {
	code unsigned char *flash = (code unsigned char *) src;
	unsigned int i;

	for(i = 0; i < size; i++) {
		*dest++ = *flash++;
	}

}

void bootBootloader() {
	unsigned char i = 0;

	CLKSEL = 0x10;
	CLKMUL = 0x00;
	OSCICN = 0x83; 				// internal oscliator / 1
	PFE0CN &= ~PFEN;			// disable prefetch

	for (i = 0; i < 100; i++);	// Wait 5us for deinitialization

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

		ljmp #0
	_endasm;
}

void pcaUpdate() {
	PCA0CN    = 0x00;					// disable PCA0
	PCA0CP0 = 0xF04B + ((unsigned int) Servo[0] << 3);
	PCA0CP1 = 0xF04B + ((unsigned int) Servo[1] << 3);
	PCA0CP2 = 0xF04B + ((unsigned int) Servo[2] << 3);
	PCA0CP3 = 0xF04B + ((unsigned int) Servo[3] << 3);
	PCA0CP4 = 0xF04B + ((unsigned int) Servo[4] << 3);
	PCA0CN    = 0x40;					// enable PCA0
}

//-----------------------------------------------------------------------------
// Interrupt Routines
//-----------------------------------------------------------------------------

void spiInterrupt(void) interrupt 6 using 2 {
	if (SPIF) {							// transfer interrupt

		// receive
		unsigned char c = SPI0DAT;
		unsigned char i;
		SPIF = 0;						// ack interrupt

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
				case (SELECT_READ | SELECT_UART1_BUFFER):
					i = (SpiCommand & 0x0F) - 1;
					SpiSize = UartRecvBuffer[i].write;
					// send size (circular buffer)
					if (SpiSize > UartRecvBuffer[i].read) {
						SPI0DAT = SpiSize - UartRecvBuffer[i].read;
					} else {
						SPI0DAT = MAX_DATA_SIZE - UartRecvBuffer[i].read + SpiSize;
					}
					break;
				case (SELECT_READ | SELECT_CHECK):
					SPI0DAT = 1; 		// send size
					break;
				case (SELECT_BOOT):
					BootBootloader = 1;
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
		case (SELECT_READ | SELECT_UART1_BUFFER):
			i = (SpiCommand & 0x0F) - 1;
			// send next UART byte
			SPI0DAT = UartRecvBuffer[i].d[UartRecvBuffer[i].read];
			// advance index in circular buffer
			UartRecvBuffer[i].read = (UartRecvBuffer[i].read + 1) & 0x1F;
			if (UartRecvBuffer[i].read == SpiSize) {
				// reset SPI command receiver
				SpiCommand = 0;
			}
			break;
		case (SELECT_READ | SELECT_ADC): // non-standard command!!!
			if (SpiSendState == 1) {
				AdcReadIndex = c;
				SPI0DAT = AdcBuffer[AdcReadIndex] >> 8;	
			} else {
				SPI0DAT = AdcBuffer[AdcReadIndex] & 0xFF;
				// reset SPI command receiver
				SpiCommand = 0;
			}
			SpiSendState++;
			break;
		case (SELECT_READ | SELECT_REGISTER): // non-standard command!!!
			SPI0DAT = getRegister(c); // send register
			// reset SPI command receiver
			SpiCommand = 0;
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
		case (SELECT_WRITE | SELECT_UART1_BUFFER):
			i = (SpiCommand & 0x0F) - 1;
			if (SpiSendState == 1) {	// size byte
				UartSendBuffer[i].size = c;
				UartSendBuffer[i].curr = 0;
				SpiSendState++;
			} else {					// data byte
				UartSendBuffer[i].d[SpiSendState-2] = c;
				SpiSendState++;
				if (SpiSendState == UartSendBuffer[i].size + 2) {
					// start sending data to UART
					if(i == 0) {
						SBUF0 = UartSendBuffer[0].d[0];
					} else {
						SBUF1 = UartSendBuffer[1].d[0];
					}
					// reset SPI command receiver;
					SpiCommand = 0;
				}
			}
			break;
		case (SELECT_WRITE | SELECT_ADC_SEQUENCE):
			AdcSequence[SpiSendState-1] = c;
			if (SpiSendState == 16) {
				AdcIndex = 0;
				// reset SPI command receiver
				SpiCommand = 0;
			} else {
				SpiSendState++;
			}
			
			break;
		case (SELECT_WRITE | SELECT_ENABLE):
			if (SpiSendState == 1) {	// size byte
				SpiSendState++;			// ignore it
			} else {					// data byte
				if (c & ENABLE_RS232) {
					UartEn = 0;
				} else {
					UartEn = 1;
				}
				if (c & ENABLE_SERVO) {
					PCA0CN    = 0x40; // enable PCA0
				} else {
					PCA0CN    = 0x00; // disable PCA0
				}
				// reset SPI command receiver;
				SpiCommand = 0;
			}
			break;
		case (SELECT_WRITE | SELECT_REGISTER):
			switch (SpiSendState) {
			case 1:	// size byte
				// ignore it
				break;
			case 2:	// data byte
				// set baud generating timer reload value
				SpiSize = c; // this is actually the register address
				break;
			default: // register value
				setRegister(SpiSize, c);
				// reset SPI command receiver
				SpiCommand = 0;
			}
			SpiSendState++;
			break;
		case (SELECT_WRITE | SELECT_SERVO):
			if (SpiSendState == 1) {	// size byte, should be 5
				SpiSendState++;
			} else {					// data byte
				Servo[SpiSendState-2] = c;
				SpiSendState++;
				if (SpiSendState == 4+2) {
					pcaUpdate();
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
			SPI0DAT = 0xAC;
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




void uart0Interrupt(void) interrupt 4 {
	if (RI0) {								// receive interrupt
		RI0 = 0;
			
		UartRecvBuffer[0].d[UartRecvBuffer[0].write] = SBUF0;
		UartRecvBuffer[0].write = (UartRecvBuffer[0].write + 1) & 0x1F;
		// generate interrupt
		SpiInterruptStatus |= INTERRUPT_UART0_RX;
		DsIrq = 1; // DS interrupt
	}

	if (TI0) {								// transmit interrupt
		TI0 = 0;							// ack interrupt
		
		UartSendBuffer[0].curr++;
		if (UartSendBuffer[0].curr == UartSendBuffer[0].size) {
			// stop sending
			UartSendBuffer[0].size = 0;
			// generate interrupt
			SpiInterruptStatus |= INTERRUPT_UART0_TX;
			DsIrq = 1; // DS interrupt
		} else {
			// send next byte
			SBUF0 = UartSendBuffer[0].d[UartSendBuffer[0].curr];
		}
	}
}

void uart1Interrupt(void) interrupt 16 {
	//Led0 = 1;

	if (SCON1 & RI1) {						// receive interrupt
		SCON1 &= ~RI1;						// ack interrupt

		UartRecvBuffer[1].d[UartRecvBuffer[1].write] = SBUF1;
		UartRecvBuffer[1].write = (UartRecvBuffer[1].write + 1) & 0x1F;
		// generate interrupt
		SpiInterruptStatus |= INTERRUPT_UART1_RX;
		DsIrq = 1; // DS interrupt
	}

	/* for some reason TI1 is never set */
	//if (SCON & TI1) {						// transmit interrupt
	else {
		SCON1 &= ~TI1;						// ack interrupt

		UartSendBuffer[1].curr++;
		if (UartSendBuffer[1].curr == UartSendBuffer[1].size) {
			// stop sending
			UartSendBuffer[1].size = 0;
			// generate interrupt
			SpiInterruptStatus |= INTERRUPT_UART1_TX;
			DsIrq = 1; // DS interrupt
		} else {
			// send next byte
			SBUF1 = UartSendBuffer[1].d[UartSendBuffer[1].curr];
		}
	}
}

void adcInterrupt() interrupt 10 {
	AdcBuffer[AdcIndex] = (((unsigned int) ADC0H) << 8) | ((unsigned int) ADC0L);
	
	AdcIndex++;
	if(AdcIndex == 16 || AdcSequence[AdcIndex] == 0xFF) {
		AdcIndex = 0;
	}

	AD0INT = 0;								// ack interrupt

	AMX0P         = AdcSequence[AdcIndex]; 	// switch to next input in sequence
}

//-----------------------------------------------------------------------------
// Main Routine
//-----------------------------------------------------------------------------

void main() {
	PCA0MD &= ~0x40;					// Disable Watchdog timer
	F1 = 0;								// Select firmware interrupts

	portInit();
	spiInit();
	uart0Init();
	//timer2Init();
	adcInit();
	//pcaInit();

	if(*((code unsigned char *) 0x83) == 0x02) { // DSerial Edge 
		clockInit48();
		uart1Init();
		Led0 = 0;
		Led1 = 1;
	} else {									// DSerial 1/2
		clockInit24();
		Led0 = 1;
		Led1 = 1;
	}

	//Led0 = 0;
	//Led1 = 1;

	EA = 1; 							// Global interrupt enable

	DsIrq = 0;

	BootBootloader = 0;

	// we don't want to boot to bootloader from an interrupt
	// so we do it here, if it's requested by DS
	for(;;) {
		if (BootBootloader) {
			bootBootloader();
		}
	}
}
