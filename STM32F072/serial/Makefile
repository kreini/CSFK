CC=gcc
CFLAGS=-Wall -pedantic -ansi -O3 -D_GNU_SOURCE

all:	serial

.PHONY: all clean

MOD_SERIAL=uart.o tokenize.o hprintf.o

uart.o: uart.c uart.h
	$(CC) $(CFLAGS) -c uart.c

hprintf.o: hprintf.c hprintf.h
	$(CC) $(CFLAGS) -c hprintf.c

tokenize.o: tokenize.c tokenize.h
	$(CC) $(CFLAGS) -c tokenize.c

serial: serial.c $(MOD_SERIAL)
	$(CC) $(CFLAGS) -o serial serial.c $(MOD_SERIAL)

clean:
	rm -f serial *.o
