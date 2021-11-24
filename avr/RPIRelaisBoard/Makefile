# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected.
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

#DEBUGUART  = y

# variants:
#Variants:
#=========
#							PA0	PA1	PA2	PA3	PA4	PA5	PA6	PA7	PB0	PB1	PB2	PB3
#							================================================================================================
#
#ATTINY84
#========
#A: T84+L298, dual Motor, single encoder			MB_E1	MA_E1	MA_A	MB_B	SCL	INT	SDA	MB_PWM	MA_B	MB_A	MA_PWM	RESET/dW
#							PCINT0	PCINT1	o	o	-	-	-	OC0B	o	o	OC0A	-
#
#B: T84+L298, dual Motor, dual encoder, 2*NAND: 		MB_E1	MA_E1	MA_A	MB_E2	SCL	INT	SDA	MB_PWM	MA_E2	MB_A	MA_PWM	RESET/dW
#							PCINT0	PCINT1	o	PCINT3	-	-	-	OC0B	PCINT8	o	OC0A	-
#
#C: T84+VNH2SP30, single Motor, dual encoder: 		LED	MA_E2	MA_A	MA_CS	SCL	INT	SDA	MA_E1	MA_B	MA_EN	MA_PWM	RESET/dW
#							o	PCINT1	o	ADC3	-	-	-	ICP	PCINT8	o	OC0A	-
#
#ATTINY841
#=========
#D: T841+L298, dual Motor, single encoder: 		MA_A	MA_PWM	MB_PWM	MB_B	SCL	INT	SDA	MB_E1	MA_B	MB_A	MA_E1	RESET/dW
#							-	T0CC0	T0CC1	o	-	-	-	ICP1	o	o	ICP2	-
#
#E: T841+L298, dual Motor, dual encoder, 2*NAND: 	MA_A	MA_PWM	MB_PWM	MB_E2	SCL	INT	SDA	MB_E1	MA_E2	MB_A	MA_E1	RESET/dW
#							-	T0CC0	T0CC1	ADC3	-	-	-	ICP1	PCINT8	o	ICP2	-
#
#F: T841+VNH2SP30, single Motor, dual encoder: 		LED	MA_PWM	MA_A	MA_CS	SCL	INT	SDA	MA_E1	MA_E2	MA_EN	MA_B	RESET/dW
#							o	T0CC0	o	ADC3	-	-	-	ICP1	PCINT8	o	o	-

#VARIANT= VARIANT_A_T84_L298_SE
VARIANT= VARIANT_D_T841_L298_SE
#VARIANT= VARIANT_D_T841_VNH2SP30_SE
#VARIANT= VARIANT_C_T84_VNH2SP30_SE

#http://www.engbedded.com/fusecalc/
#FUSES      = -U lfuse:w:0xd2:m -U hfuse:w:0xde:m -U efuse:w:0xfe:m
#FUSES      = -U lfuse:w:0xd2:m -U hfuse:w:0x9e:m -U efuse:w:0xfe:m
T841_DW_FUSES      = -U lfuse:w:0xe2:m -U hfuse:w:0x9f:m -U efuse:w:0xff:m

DEVICE     = attiny84
TWI_ADDR   = 10

CFLAGS += -D$(VARIANT)

CLOCK      = 8000000
PROGRAMMER = -c usbtiny
#PROGRAMMER = -c stk500 -P /dev/ttyACM0
OBJECTS    = main.o 

ifeq ($(DEVICE), attiny84)
OBJECTS += ../lib/i2c.o
endif

ifeq ($(DEVICE), attiny841)
OBJECTS += ../lib/i2c_tws.o
endif

ifdef DEBUGUART
CFLAGS += -DDEBUGUART
endif

######################################################################
######################################################################

# Tune the lines below only if you know what you are doing:

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -Wall -Os ${CFLAGS} -DTWI_ADDR=$(TWI_ADDR) -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -I../lib -I../pid_autotuner

# symbolic targets:
all:	main.hex

dw: clean all
	dwdebug device usbtiny,l main.elf,qr

dwr: 
	dwdebug device usbtiny,reset,qr
dwq: 
	dwdebug device usbtiny,qi

.c.o:
	$(COMPILE) -c $< -o $@

.cpp.o:
	$(COMPILE) -c $< -o $@ -std=gnu++11

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

flash_only:
	$(AVRDUDE) -U flash:w:main.hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash fuse

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(OBJECTS)
	$(COMPILE) -o main.elf $(OBJECTS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
