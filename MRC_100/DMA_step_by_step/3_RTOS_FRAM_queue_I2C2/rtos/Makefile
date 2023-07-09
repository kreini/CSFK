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
OCD_PRG=stlink-v2
OCD_MCU=stm32f0x

OCD_CFG  = -f interface/$(OCD_PRG).cfg 
OCD_CFG += -c "transport select hla_swd"
OCD_CFG += -f target/$(OCD_MCU).cfg -c "adapter_khz 100"
OCD_CFG += -c "reset_config srst_only srst_nogate"

DEPTH=..

FREERTOS=$(DEPTH)/freertos
ARM=$(DEPTH)/arm

CFLAGS  = -fno-common -mcpu=$(ARM_CPU) -march=$(ARM_ARCH) -mthumb -masm-syntax-unified -D$(STM) -Wall -Wno-pointer-sign -O3
#CFLAGS += -I$(ARM)/include
#CFLAGS += -I$(FREERTOS)/include -I.
LFLAGS  = -Tmain.ld -nostartfiles
BFLAGS  = -Tbootloader.ld -nostartfiles

MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/$(GCC_ARCH)/libm.a
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/$(GCC_ARCH)/libc_nano.a 
MAIN_LIBS += /usr/lib/gcc/arm-none-eabi/$(GCC_VERS)/$(GCC_ARCH)/libgcc.a 
MAIN_LIBS += /usr/lib/arm-none-eabi/newlib/$(GCC_ARCH)/libnosys.a

FREERTOS_MODULES += list.o tasks.o queue.o timers.o port.o heap_4.o 
FREERTOS_LIB = libfreertos.a

.PHONY: all clean upload bootupload boot

all: $(FREERTOS_LIB)

heap_3.o: heap_3.c
	$(CC) $(CFLAGS) -c heap_3.c

heap_4.o: heap_4.c
	$(CC) $(CFLAGS) -c heap_4.c

list.o: list.c
	$(CC) $(CFLAGS) -c list.c

tasks.o: tasks.c
	$(CC) $(CFLAGS) -c tasks.c

queue.o: queue.c
	$(CC) $(CFLAGS) -c queue.c

timers.o: timers.c
	$(CC) $(CFLAGS) -c timers.c

port.o: port.c
	$(CC) $(CFLAGS) -c port.c

libfreertos.a: $(FREERTOS_MODULES)
	$(AR) src libfreertos.a $(FREERTOS_MODULES)

clean:
	rm -f *.o *.a *.elf *.bin *.hex *.disasm
