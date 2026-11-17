# openHCOP firmware simulation tests (simavr)
#
#   make        build firmware from ../src/main.c, build bench, run tests
#   make v      same, with every output edge printed
#   make clean
#
# Needs: gcc-avr avr-libc simavr libsimavr-dev  (Debian/Ubuntu package names)

SIMAVR_INC ?= /usr/include/simavr
CC         ?= gcc
AVRCC      ?= avr-gcc

FW_SRC = ../src/main.c
FW_ELF = firmware.elf
# Same code generation options as the PlatformIO / src/Makefile build
FW_CFLAGS = -g -DF_CPU=8000000UL -mmcu=attiny44 -std=gnu11 -Os -Wall -Wno-comment \
            -ffunction-sections -fdata-sections -flto -Wl,--gc-sections

all: test

$(FW_ELF): $(FW_SRC)
	$(AVRCC) $(FW_CFLAGS) -o $@ $<

harness: harness.c
	$(CC) -O1 -Wall -I$(SIMAVR_INC) -o $@ $< -lsimavr

test: harness $(FW_ELF)
	./harness $(FW_ELF)

v: harness $(FW_ELF)
	./harness $(FW_ELF) -v

clean:
	rm -f harness $(FW_ELF)

.PHONY: all test v clean
