#!/bin/bash

./simarmv6m \
	--memory 0x08000000:0:main.bin:+progmem \
	--memory 0x20000000:16384:+ram \
	--peripheral stm32f0/usart:base=0x40004400,interrupt=28,device=/dev/tnt1 \
	--cycles 0



