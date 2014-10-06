CC	:= avr-gcc
LD	:= avr-ld
OBJCOPY	:= avr-objcopy
OBJDUMP	:= avr-objdump
SIZE	:= avr-size

TARGET = uartboot
SOURCE = $(wildcard *.c)

#CONFIG = ispprog
#CONFIG = flightctrl
#CONFIG = funkstuff
CONFIG = ispprog2

AVRDUDE_PROG := -c avr910 -b 115200 -P /dev/ttyUSB0
#AVRDUDE_PROG := -c dragon_isp -P usb

# ---------------------------------------------------------------------------

ifeq ($(CONFIG), ispprog)
MCU=atmega16
AVRDUDE_MCU=m16

# (7.3728MHz ext. crystal)
AVRDUDE_FUSES=lfuse:w:0xff:m hfuse:w:0xda:m
BOOTLOADER_START=0x3C00
endif

# -------------------------

ifeq ($(CONFIG), flightctrl)
MCU=atmega644p
AVRDUDE_MCU=m644p

# (20MHz ext. Crystal, 2.7V BOD)
AVRDUDE_FUSES=lfuse:w:0xff:m hfuse:w:0xdc:m efuse:w:0xfd:m
BOOTLOADER_START=0xF800
endif

# -------------------------

ifeq ($(CONFIG), funkstuff)
MCU=atmega88
AVRDUDE_MCU=m88 -F

# (ext. crystal)
#AVRDUDE_FUSES=lfuse:w:0x84:m hfuse:w:0xda:m
BOOTLOADER_START=0x1C00
endif

# -------------------------

ifeq ($(CONFIG), ispprog2)
MCU=atmega328p
AVRDUDE_MCU=m328p -F

# (8MHz internal osc., 2.7V BOD)
AVRDUDE_FUSES=lfuse:w:0xe2:m hfuse:w:0xdc:m efuse:w:0x02:m
BOOTLOADER_START=0x7C00
endif

# ---------------------------------------------------------------------------

CFLAGS = -pipe -g -Os -mmcu=$(MCU) -Wall
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -Wa,-adhlns=$(*F).lst
CFLAGS += -DBOOTLOADER_START=$(BOOTLOADER_START) -DCONFIG_$(CONFIG)=1
LDFLAGS = -Wl,-Map,$(@:.elf=.map),--cref,--section-start=.text=$(BOOTLOADER_START)
LDFLAGS += -Wl,--relax,--gc-sections

LDSCRIPT := $(shell LANG=C $(CC) $(CFLAGS) -Wl,--verbose 2> /dev/null | awk '/^opened script file (.*)$$/{ print $$4 }')
LDSCRIPT_NOVECT := ldscript-no-vectors-$(notdir $(LDSCRIPT))

# ---------------------------------------------------------------------------

$(TARGET): $(TARGET).elf
	@$(SIZE) -B -x --mcu=$(MCU) $<

$(TARGET).elf: $(SOURCE:.c=.o) | $(LDSCRIPT_NOVECT)
	@echo " Linking file:  $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -Wl,-T$(LDSCRIPT_NOVECT) -o $@ $^ #2> /dev/null
	@$(OBJDUMP) -h -S $@ > $(@:.elf=.lss)
	@$(OBJCOPY) -j .text -j .data -O ihex $@ $(@:.elf=.hex)
	@$(OBJCOPY) -j .text -j .data -O binary $@ $(@:.elf=.bin)

%.o: %.c $(MAKEFILE_LIST)
	@echo " Building file: $<"
	@$(CC) $(CFLAGS) -o $@ -c $<

# remove interrupt vector section from avr-libc linker script
# (remove all lines with *vectors* and insert DISCARD line above .text declaration)
$(LDSCRIPT_NOVECT): $(LDSCRIPT) $(MAKEFILE_LIST)
	@echo " Creating:      $@"
	@sed -e '/.*vectors.*/d' -e 's/\(^[ \t]*\)\(.text[ \t]*\:\)$$/\1\/DISCARD\/ : { *(.vectors) }\n\1\2/g' $< > $@
#	-@diff -uNr $< $@ > $@.diff

clean:
	rm -rf $(SOURCE:.c=.o) $(SOURCE:.c=.lst) $(addprefix $(TARGET), .elf .map .lss .hex .bin) ldscript-no-vectors-*

install: $(TARGET).elf
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) -U flash:w:$(<:.elf=.hex)

fuses:
	avrdude $(AVRDUDE_PROG) -p $(AVRDUDE_MCU) $(patsubst %,-U %, $(AVRDUDE_FUSES))
