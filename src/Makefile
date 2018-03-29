# Name: Makefile
# Author: icelord
# Copyright: 2018
# License: GNU Public license V3

DEVICE     = attiny44
CLOCK      = 8000000L
PROGRAMMER = -Pusb
OBJECTS    = main.o
FUSES      = -U lfuse:w:0x52:m -U hfuse:w:0xdf:m
CFLAGS     = -std=gnu11 -fno-fat-lto-objects -g -Os -Wall -ffunction-sections -fdata-sections -flto -Wno-comment -DARDUINO_AVR_ATTINYX4
AVRPATH    = 


######################################################################
######################################################################

# Tune the lines below only if you know what you are doing:
AVRDUDE = $(AVRPATH)avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = $(AVRPATH)avr-gcc -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) $(CFLAGS)

## symbolic targets:
all:	main.hex
.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash fuse

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	$(AVRPATH)avr-objcopy -j .text -j .data -O ihex main.elf main.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	$(AVRPATH)avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
