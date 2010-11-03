/*-------------------------------------------------------------------------------
	DSerial Simple Console Example
-------------------------------------------------------------------------------*/

#include <nds.h>
#include <stdio.h>
#include <string.h>

/* DSerial includes */
#include "dserial.h"
#include "firmware_bin.h"


/*-------------------------------------------------------------------------------*/
void keycodeHandler(int keycode) {
/*-------------------------------------------------------------------------------*/
	if (keycode > 0) {
		char c = (char) keycode;
		iprintf("%c", c);						/* local echo */
		dseUartSendBuffer(UART0, &c, 1, true);	/* blocking uart0 */
		dseUartSendBuffer(UART1, &c, 1, true);	/* blocking uart1 */
	}
}

/*-------------------------------------------------------------------------------*/
int main(void) {
/*-------------------------------------------------------------------------------*/
	/* Setup console on top screen and keyboard on bottom screen */
	
	videoSetMode(MODE_0_2D);
	videoSetModeSub(MODE_0_2D);
	vramSetBankA(VRAM_A_MAIN_BG);
	vramSetBankC(VRAM_C_SUB_BG);

    consoleInit(NULL , 0, BgType_Text4bpp, BgSize_T_256x256, 22, 3, true, true);

	Keyboard *keyboard = keyboardGetDefault();
	keyboard->OnKeyPressed = keycodeHandler;
	//used in r24 keyboardInit(keyboard);
    keyboardDemoInit();

	iprintf("-- DSerial Console --\n\n");

	while(!dseInit()) {
		iprintf("DSerial is not inserted!\n");
		iprintf("Insert DSerial and press A.\n");

		while(~keysDown() & KEY_A) { swiWaitForVBlank(); scanKeys(); }
		while(~keysUp() & KEY_A) { swiWaitForVBlank(); scanKeys(); }
	}
	iprintf("DSerial initialized!\n");
	int version = dseVersion();
	if(version < 2) {
		printf("Version: DSerial1/2\n");
	} else if(version == 2) {
		printf("Version: DSerial Edge\n");
	}

	/* Now see if the firmware needs to be updated and update if necessary */

	if (dseMatchFirmware((char *) firmware_bin, firmware_bin_end - firmware_bin)) {
		iprintf("Firmwares match!\n");
	} else {
		iprintf("Updating firmware...\n");
		dseUploadFirmware((char *) firmware_bin, firmware_bin_end - firmware_bin);
		iprintf("Done!\n");
	}
	dseBoot();
	swiDelay(9999); /* Wait for the FW to boot */
	if (dseStatus() == FIRMWARE) {
		iprintf("Firmware booted successfully!\n");
	} else {
		iprintf("Failed to boot firmware!\n");
	}

	/*  Set DSerial modes 

		DSerial can interface with RS232 and TTL/CMOS level UART ports.
		If you need RS232, then set ENABLE_RS232 and wire up to RS232 connector.
		If you need CMOS/TTL, then set ENABLE_CMOS and wire up to IO2 connector.

		See http://www.beyondlogic.org/serial/serial1.htm#40 for explanation of
		the difference between RS232 and CMOS/TTL levels.

		Note: This doesn't apply to DSerial Edge which lacks RS232 converter
	*/
	dseSetModes(ENABLE_RS232);

	/* Now DSerial is ready to be used */

	/*	You can set your own UART receive handler that will be called on an
		interrupt.

		See dseUartDefaultReceiveHandler() in dserial.c for an example.
	*/
	/* dseUartSetReceiveHandler(yourOwnUartHandler); */

	iprintf("\n");
	iprintf("A - Sends \"Hello\" to UARTs\n");
	iprintf("B - N/A\n");
	iprintf("X - Sets UART baud rate to 4800\n");
	iprintf("Y - N/A\n");
	iprintf("L - N/A\n");
	iprintf("R - Bootloader <--> Firmware\n");
	iprintf("\n");

	keyboardShow();

	while(1) {
		swiWaitForVBlank();
		//used in r24 keyboardUpdate();
		scanKeys();

		if(keysDown() & KEY_A) {
			iprintf("<Hello>");
			dseUartSendBuffer(UART0, "Hello!", 6, true); /* blocking UART0 send */
			dseUartSendBuffer(UART1, "Hello!", 6, true); /* blocking UART1 send */
		} else if(keysDown() & KEY_B) {
			iprintf("<B>");
		} else if(keysDown() & KEY_X) {
			iprintf("<4800>");
			dseUartSetBaudrate(UART0, 4800); /* at boot-up, UART0 baudrate is set to 115200 bps */
			dseUartSetBaudrate(UART1, 4800); /* at boot-up, UART1 baudrate is set to 9600 bps */
		} else if(keysDown() & KEY_Y) {
			iprintf("<Y>");
		} else if(keysDown() & KEY_L) {
			iprintf("<L>");
		} else if(keysDown() & KEY_R) {
			iprintf("<Boot>");
			dseBoot();
		}
	}

	return 0;
}
