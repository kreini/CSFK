SHELL=/bin/sh

CC=gcc
AR=ar
LD=ld

CFLAGS=-Wall -pedantic -ansi -O3 -D_GNU_SOURCE -fPIC

.PHONY: all clean

TARGETS=simarmv6m
MOD_SIMARMV6M=timeval.o stm32f0_usart.o armv6m.o memory.o

all: $(TARGETS)

timeval.o: timeval.c timeval.h
	$(CC) $(CFLAGS) -c timeval.c

armv6m.o: armv6m.c armv6m.h
	$(CC) $(CFLAGS) -c armv6m.c

memory.o: memory.c memory.h
	$(CC) $(CFLAGS) -c memory.c

stm32f0_usart.o: stm32f0_usart.c stm32f0_usart.h
	$(CC) $(CFLAGS) -c stm32f0_usart.c

simarmv6m: simarmv6m.c $(MOD_SIMARMV6M) 
	$(CC) $(CFLAGS) -o simarmv6m simarmv6m.c $(MOD_SIMARMV6M)


clean:
	rm -f *.o $(TARGETS)
