# =============================================================================
# Top-level Makefile - Generic for STM32CubeMX CMake projects
# Project name is read automatically from CMakeLists.txt
# Flash: uses stm32flash via STM32 UART bootloader (requires BOOT0 = 1)
# =============================================================================

# --- Auto-detect project name ------------------------------------------------
PROJECT_NAME := $(shell grep -oE 'CMAKE_PROJECT_NAME\s+\w+' CMakeLists.txt | awk '{print $$2}')

BUILD_DIR  := build
TARGET     := $(BUILD_DIR)/$(PROJECT_NAME)

# --- Tools -------------------------------------------------------------------
OBJCOPY    := arm-none-eabi-objcopy

# Override with: make flash STMFLASH=/path/to/stm32flash.exe
STMFLASH   ?= D:/下载/STM32L051/stm32flash/stm32flash.exe

# Serial port / baud - override with: make flash PORT=COM5
PORT       ?= COM3
BAUD       ?= 115200

# Flash start address for all STM32 devices
FLASH_ADDR := 0x08000000

# -----------------------------------------------------------------------

.PHONY: all build bin flash erase clean help

all: bin

## Build the ELF via cmake-generated Makefiles
build:
	$(MAKE) -C $(BUILD_DIR)

## Convert ELF -> BIN
bin: build
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin
	@echo "Binary ready: $(TARGET).bin"

## Flash BIN to MCU over UART bootloader
## Before running: set BOOT0=1 and reset the MCU
flash: bin
	$(STMFLASH) -b $(BAUD) -w $(TARGET).bin -v -g $(FLASH_ADDR) $(PORT)

## Erase flash only (no write)
erase:
	$(STMFLASH) -b $(BAUD) -o $(PORT)

## Remove build artifacts (keeps cmake cache)
clean:
	$(MAKE) -C $(BUILD_DIR) clean
	rm -f $(TARGET).bin $(TARGET).hex $(TARGET).map

help:
	@echo "Project : $(PROJECT_NAME)"
	@echo ""
	@echo "Targets:"
	@echo "  all      - Build ELF and convert to BIN (default)"
	@echo "  build    - Build ELF only"
	@echo "  bin      - Build ELF and convert to BIN"
	@echo "  flash    - Flash BIN via UART bootloader"
	@echo "  erase    - Erase flash only"
	@echo "  clean    - Clean build artifacts"
	@echo ""
	@echo "Overrides:"
	@echo "  make flash PORT=COM5"
	@echo "  make flash PORT=COM3 BAUD=57600"
	@echo "  make flash STMFLASH=/path/to/stm32flash.exe PORT=COM3"
	@echo ""
	@echo "Before flashing: set BOOT0=1, reset MCU, then run 'make flash'"
