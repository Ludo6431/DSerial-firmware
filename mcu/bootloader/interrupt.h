/*
	DSerial Bootloader v1.0
	Copyright (C) 2008 Alexei Karpenko

	This program is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
	General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

#define IRQ_X0		0x03	/* External Interrupt 0 (/INT0) */
#define IRQ_T0		0x0B	/* Timer 0 Overflow */
#define IRQ_X1		0x13	/* External Interrupt 1 (/INT1) */
#define IRQ_T1		0x1B	/* Timer 1 Overflow */
#define IRQ_S0		0x23	/* UART0 */
#define IRQ_T2		0x2B	/* Timer 2 Overflow */
#define IRQ_SPI0	0x33	/* SPI0 */
#define IRQ_SMB0	0x3B	/* SMB0 */
#define IRQ_USB0	0x43	/* USB0 */
#define IRQ_WADC0	0x4B	/* ADC0 Window Compare */
#define IRQ_ADC0	0x53	/* ADC0 Conversion Complete */
#define IRQ_PCA0	0x5B	/* Programmable Counter Array */
#define IRQ_CP0		0x63	/* Comparator0 */
#define IRQ_CP1		0x6B	/* Comparator1 */
#define IRQ_T3		0x73	/* Timer 3 Overflow */
#define IRQ_VBUS	0x7B	/* VBUS Level */
#define IRQ_S1		0x83	/* UART1 */

//-----------------------------------------------------------------------------
// Interrupt Redirection
//-----------------------------------------------------------------------------

/* External Interrupt 0 (/INT0) */
void redirX0() __interrupt (0) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_X0
	_endasm;
}

/* Timer 0 Overflow */
void redirT0() __interrupt (1) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_T0
	_endasm;
}

/* External Interrupt 1 (/INT1) */
void redirX1() __interrupt (2) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_X1
	_endasm;
}

/* Timer 1 Overflow */
void redirT1() __interrupt (3) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_T1
	_endasm;
}

/* UART0 */
void redirS0() __interrupt (4) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				lcall _uartInterrupt
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_S0
	_endasm;
}

/* Timer 2 Overflow */
void redirT2() __interrupt (5) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_T2
	_endasm;
}

/* SPI0 */
void redirSPI0() __interrupt (6) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				lcall _spiInterrupt
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_SPI0
	_endasm;
}

/* SMB0 */
void redirSMB0() __interrupt (7) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_SMB0
	_endasm;
}

/* USB0 */
void redirUSB0() __interrupt (8) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_USB0
	_endasm;
}

/* ADC0 Window Compare */
void redirWADC0() __interrupt (9) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_WADC0
	_endasm;
}

/* ADC0 Conversion Complete */
void redirADC0() __interrupt (10) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_ADC0
	_endasm;
}

/* Programmable Counter Array */
void redirPCA0() __interrupt (11) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_PCA0
	_endasm;
}

/* Comparator0 */
void redirCP0() __interrupt (12) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_CP0
	_endasm;
}

/* Comparator1 */
void redirCP1() __interrupt (13) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_CP1
	_endasm;
}

/* Timer 3 Overflow */
void redirT3() __interrupt (14) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_T3
	_endasm;
}

/* VBUS Level */
void redirVBUS() __interrupt (15) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_VBUS
	_endasm;
}

/* UART1 */
void redirS1() __interrupt (16) _naked {
	_asm
				push psw
				jnb  psw.1, 00001$
		
				pop  psw
/*				lcall _uartInterrupt1 */
				reti
		
		00001$:
				pop  psw
				ljmp #FIRMWARE_OFFSET + IRQ_S1
	_endasm;
}

