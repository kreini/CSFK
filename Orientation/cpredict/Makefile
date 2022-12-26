SHELL=/bin/sh

CC=gcc
AR=ar
LD=ld
CFLAGS=-Wall -pedantic -ansi -O3 -D_GNU_SOURCE -fPIC

TARGETS=cpredict

.PHONY: all clean

all: $(TARGETS)

predict.o: predict.c predict.h
	$(CC) $(CFLAGS) -c predict.c 

cpredict: cpredict.c predict.o
	$(CC) $(CFLAGS) -o cpredict cpredict.c predict.o -lm

clean:
	rm -f *.o $(TARGETS)


