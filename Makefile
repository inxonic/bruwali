# Copying and distribution of this file, with or without modification,
# are permitted in any medium without royalty provided the copyright
# notice and this notice are preserved.  This file is offered as-is,
# without any warranty.


MCU=		atmega48p
DPROG=		usbtiny

ISP_TARGET=	bruwali.ihx

TARGETS=	$(ISP_TARGET)
OBJECTS=	bruwali.o

CC=		avr-gcc

CFLAGS=		-mmcu=$(MCU) -Os -std=c99
LDFLAGS=	-mmcu=$(MCU)

OBJCOPY=	avr-objcopy
OBJCOPYFLAGS=	-R .eeprom

AVRDUDE=	avrdude
AVRDUDEFLAGS=	-p $(MCU) -c $(DPROG) -B 16kHz


all:		$(TARGETS)


bruwali.elf:	$(OBJECTS)


%.elf : %.o
		$(CC) $(LDFLAGS) -o $@ $^

%.ihx : %.elf
		$(OBJCOPY) $(OBJCOPYFLAGS) -O ihex $< $@

isp:	$(ISP_TARGET)
		$(AVRDUDE) $(AVRDUDEFLAGS) -U flash:w:$(ISP_TARGET):i

clean:
		rm -f $(TARGETS) *.elf *.o
