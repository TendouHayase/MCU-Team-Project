CC := avr-g++
CFLAGS := -Wall -Wextra -Os -mmcu=atmega328p 
AVRDUDE_FLAGS := -c arduino -p m328p -b 115200
AVROBJCP_FLAGS := -O ihex -R .eeprom
OBJDUMP := avr-objdump
AVRDUDE := avrdude
AVROBJCP := avr-objcopy

ifeq ($(src),)
    $(error source file named src is required)
endif
ifneq (,$(filter upload all,$(MAKECMDGOALS)))
    ifeq ($(port),)
        $(error port is required)
    endif
endif




PORT := $(port)
SRC := $(src)
TARGET := $(SRC:.c=)
TARGET_ELF := $(TARGET).elf
TARGET_HEX := $(TARGET).hex

compile: $(TARGET_HEX)
all: upload

$(TARGET_ELF): $(SRC)
	$(CC) $(CFLAGS) $< -o $@

$(TARGET).lst: $(TARGET_ELF)
	$(OBJDUMP) -d $< > $@

$(TARGET_HEX): $(TARGET_ELF)
	$(AVROBJCP) $(AVROBJCP_FLAGS) $< $@


upload: $(TARGET_HEX)
	$(AVRDUDE) $(AVRDUDE_FLAGS) -P $(PORT) -U flash:w:$(TARGET_HEX):i

disasm: $(TARGET).lst

.PHONY: all compile disasm upload
