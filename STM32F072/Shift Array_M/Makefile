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
STM=STM32F072

OCD=openocd
OCD_PRG=stlink-v2-1
OCD_MCU=stm32f0x

OCD_CFG  = -f interface/$(OCD_PRG).cfg 
OCD_CFG += -c "transport select hla_swd"
OCD_CFG += -f target/$(OCD_MCU).cfg -c "adapter_khz 100"
OCD_CFG += -c "reset_config srst_only srst_nogate"

ARM=/usr/lib/arm-none-eabi

CFLAGS  = -fno-common -mcpu=$(ARM_CPU) -march=$(ARM_ARCH) -mthumb -masm-syntax-unified -D$(STM) -Wall -Wno-pointer-sign -O3
CFLAGS += -I$(ARM)/include
CFLAGS += -mfloat-abi=soft
LFLAGS  = -Tmain.ld -nostartfiles
LFLAGS += -u _printf_float
BFLAGS  = -Tbootloader.ld -nostartfiles

MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/thumb/v6-m/nofp/libc_nano.a
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/thumb/v6-m/nofp/libm.a
MAIN_LIBS += /usr/lib/gcc/arm-none-eabi/9.2.1/thumb/v6-m/nofp/libgcc.a
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/thumb/v6-m/nofp/libnosys.a

MAIN_MODULES += stm32_rcc.o 

BOOTLOADER_MODULES += stm32_rcc.o 

.PHONY: all clean upload 

all: main.hex main.disasm main.bin

crt0.o: crt0.c
	$(CC) $(CFLAGS) -c crt0.c

stm32_rcc.o: stm32_rcc.c stm32_rcc.h
	$(CC) $(CFLAGS) -c stm32_rcc.c

main.o: main.c config.h
	$(CC) $(CFLAGS) -c main.c

main.elf: crt0.o main.o $(MAIN_MODULES)
	$(LD) $(LFLAGS) -o main.elf crt0.o main.o $(MAIN_MODULES) $(MAIN_LIBS)

main.hex: main.elf
	$(OBJCOPY) -O ihex main.elf main.hex 
	$(SREC_CAT) main.hex -I -crop 0x00000000 0x1FFFFFFF -o main.hex -I

main.bin: main.hex
	$(SREC_CAT) main.hex -I -offset -0x08000000 -o main.bin -binary

main.ihex: main.bin
	$(SREC_CAT) main.bin -binary -o main.ihex -I

main.disasm: main.elf
	$(OBJDUMP) -s -d main.elf > main.disasm

upload: main.hex
	$(OCD) $(OCD_CFG) -c init -c "reset halt" -c "stm32f0x unlock 0" -c "reset halt" -c "flash write_image erase main.hex 0 ihex" -c "reset" -c "exit" || exit 0

reset: 
	$(OCD) $(OCD_CFG) -c init -c reset -c exit || exit 0

clean:
	rm -f *.o *.elf *.bin *.hex *.disasm
