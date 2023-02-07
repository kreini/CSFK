SHELL=/bin/sh

CC=arm-none-eabi-gcc
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump
SREC_CAT=srec_cat
SREC_INFO=srec_info

ARM_CPU=cortex-m0
ARM_ARCH=armv6-m
STM=STM32F042

gcc_version=$(shell $(CC) --version | head -n 1 | awk '{ print $$3; }')

ifeq ($(gcc_version),5.4.1)
GCC_ARCH=armv6-m
GCC_VERS=5.4.1
else ifeq ($(gcc_version),8.3.1)
GCC_ARCH=thumb/v6-m/nofp
GCC_VERS=8.3.1
else
GCC_ARCH=thumb/v6-m/nofp
GCC_VERS=$(gcc_version)
endif

ifndef GCC_VERS
$(error GCC_VERS is not set)
endif

OCD=openocd
OCD_PRG=stlink-v2-1
OCD_MCU=stm32f0x

OCD_CFG  = -f interface/$(OCD_PRG).cfg 
OCD_CFG += -c "transport select hla_swd"
OCD_CFG += -f target/$(OCD_MCU).cfg -c "adapter_khz 100"
OCD_CFG += -c "reset_config srst_only srst_nogate"

DEPTH=..

FREERTOS=$(DEPTH)/freertos
## !!!
ARM=/usr/local/stm32_headers
#ARM=/usr/local/arm-none-eabi

CFLAGS  = -fno-common -mcpu=$(ARM_CPU) -march=$(ARM_ARCH) -mthumb -masm-syntax-unified -D$(STM) -Wall -Wno-pointer-sign -O3 -D_GNU_SOURCE
CFLAGS += -I$(ARM)/include
LFLAGS  = -Tmain.ld -nostartfiles
BFLAGS  = -Tbootloader.ld -nostartfiles

## !!! 
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/$(GCC_ARCH)/libm.a
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/$(GCC_ARCH)/libc_nano.a 
MAIN_LIBS += /usr/lib/gcc/arm-none-eabi/$(GCC_VERS)/$(GCC_ARCH)/libgcc.a 
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/$(GCC_ARCH)/libnosys.a

#MAIN_MODULES += stm32_bxcan.o stm32_spi.o stm32_i2c_dma.o stm32_crc.o stm32_rcc.o
MAIN_MODULES += stm32_rcc.o stm32_i2c_dma.o
MAIN_MODULES += rtos/libfreertos.a

.PHONY: all clean upload rtos

all: rtos main.hex main.disasm main.bin 

rtos:
	$(MAKE) -C rtos

crt0.o: crt0.c
	$(CC) $(CFLAGS) -c crt0.c

#stm32_bxcan.o: stm32_bxcan.c stm32_bxcan.h
#	$(CC) $(CFLAGS) -c stm32_bxcan.c

#stm32_crc.o: stm32_crc.c stm32_crc.h
#	$(CC) $(CFLAGS) -c stm32_crc.c

stm32_rcc.o: stm32_rcc.c stm32_rcc.h
	$(CC) $(CFLAGS) -c stm32_rcc.c

#stm32_spi.o: stm32_spi.c stm32_spi.h
#	$(CC) $(CFLAGS) -c stm32_spi.c

stm32_i2c_dma.o: stm32_i2c_dma.c stm32_i2c_dma.h
	$(CC) $(CFLAGS) -c stm32_i2c_dma.c

#stm32_flash.o: stm32_flash.c stm32_flash.h
#	$(CC) $(CFLAGS) -c stm32_flash.c

#stm32_watchdog.o: stm32_watchdog.c stm32_watchdog.h
#	$(CC) $(CFLAGS) -c stm32_watchdog.c

main.o: main.c 
	$(CC) $(CFLAGS) -c main.c

main.elf: crt0.o main.o $(MAIN_MODULES)
	$(LD) $(LFLAGS) -o main.elf crt0.o main.o $(MAIN_MODULES) $(MAIN_LIBS)

main.hex: main.elf
	$(OBJCOPY) -O ihex main.elf main.hex 
	$(SREC_CAT) main.hex -I -crop 0x00000000 0x1FFFFFFF -o main.hex -I

main.bin: main.hex
	$(SREC_CAT) main.hex -I -offset -0x08000000 -o main.bin -binary

main.disasm: main.elf
	$(OBJDUMP) -s -d main.elf > main.disasm

upload: main.hex
	$(OCD) $(OCD_CFG) -c init -c "reset halt" -c "stm32f0x unlock 0" -c "reset halt" -c "flash write_image erase main.hex 0 ihex" -c "reset" -c "exit" || exit 0

reset: 
	$(OCD) $(OCD_CFG) -c init -c reset -c exit || exit 0

clean:
	$(MAKE) -C rtos clean
	rm -f *.o *.elf *.bin *.hex *.disasm
