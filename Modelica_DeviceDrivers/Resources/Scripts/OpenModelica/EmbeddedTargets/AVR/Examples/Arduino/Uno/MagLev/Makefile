###############################################################################
# (LINUX) Makefile for building Modelica_DeviceDrivers.EmbeddedTargets.AVR.Examples.Arduino.UNO.MagLev
#
# Flash example:
#   make flash
#
###############################################################################

include ../../../../Makefile.inc

MCU = atmega328p

CFLAGS += -mmcu=$(MCU)

LDFLAGS += -mmcu=$(MCU)

OBJ=MagLev_main.o \

.PHONY: clean flash

all: MagLev_main.elf MagLev_main.hex

MagLev_main.c: MagLev.mos
	omc --simCodeTarget=ExperimentalEmbeddedC MagLev.mos

MagLev_main.o: MagLev_main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

MagLev_main.elf: $(OBJ)
	$(CC) $(LDFLAGS) $^ -o $@

MagLev_main.hex: MagLev_main.elf
	avr-objcopy -O ihex -R .eeprom $< $@

# flash to target. Adapt settings as needed
flash: MagLev_main.hex
	avrdude -F -V -c arduino -p ATMEGA328P -P /dev/ttyACM0 -b 115200 -U flash:w:$<

clean:
	@$(RM) MagLev_main.o
	@$(RM) MagLev_main.*
