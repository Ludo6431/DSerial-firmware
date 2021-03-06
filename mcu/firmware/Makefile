# DSerial Firmware Makefile

# Additional directories to search for include files
INCLUDES := -I.

# Flags provided to SDCC compiler
CFLAGS := -c -V $(INCLUDES)

# Flags provided to SDCC linker, Firmware located at 0x0800
LFLAGS := -V --code-loc 0x0800

# Default target to build
all: firmware.bin

# Object files to be linked separated by spaces
OBJECTS := firmware.rel registers.rel

.SUFFIXES: .hex .bin

# Builds RAW binary of firmware from intel hex format file
firmware.bin: firmware.hex
	ihex2bin -o $*.bin $*.hex

# Links and builds the target hex file
firmware.hex: $(OBJECTS)
	sdcc $(LFLAGS) $^
	packihx $*.ihx > $*.hex

# Compiles c files
%.rel: %.c
	sdcc $(CFLAGS) $<

# Compiles asm files (excluding asm files generated by SDCC)
%.rel: %.asm
	asx8051 -losa $<

# Cleans temporary files, leaves hex and bin file
clean:
	rm -f *.rel *.lst *.lnk *.rst *.mem *.map *.sym *.ihx
