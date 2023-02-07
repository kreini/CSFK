#!/bin/bash

trap 'echo -e "\033[0m"' SIGINT

if [ -n "$1" ]; then
	replay_file="$1"
fi

#function clear()
#{
# echo -e -n "\033[H\033[2J"
#}

offset=900
range=1000

color_map=(0,0,0
      0,0,51
      0,0,102
      0,0,153
      0,0,204
      0,0,255
      0,51,255
      0,102,255
      0,153,255
      0,204,255
      0,255,255
      0,255,204
      0,255,153
      0,255,102
      0,255,51
      0,255,0
      51,255,0
      102,255,0
      153,255,0
      204,255,0
      255,255,0
      255,204,0
      255,153,0
      255,102,0
      255,51,0
      255,0,0
      255,51,51
      255,102,102
      255,153,153
      255,204,204
      255,255,255)

ncolor=${#color_map[*]}

reset
clear

date=`date +%Y%m%d-%H%M%S`

if test -n "$replay_file" && test -f "$replay_file"; then
	cat "$replay_file" 
else
	/home/kreinicker/Desktop/CSFK/STM32F072/serial/serial -c /dev/ttyACM0 -b 921600 | \
	awk -v is_print=0 \
	 '{	if ( $1=="---end---" && NF==1 )	
			is_print++;
		if ( 2<=is_print )
		 {	print;fflush();
		 }
	 }' | \
	tee hex2img-${date}.txt 
fi | \
while read line; do
	A=($line)
	if [ "${A[0]}" == "---end---" ]; then
		echo -n -e "\033[H"
	elif [ ${#A[*]} == 16 ]; then
		for ((j=0;j<2;j++)); do for ((i=0;i<16;i++)); do
			c=${A[i]}
			c=$((0x${c}))
			[ $((0x8000)) -le $c ] && c=$((c-0x10000))
			c=$((c+offset))
			c=$((c<0?0:range<=c?range-1:c))
			c=$((c*ncolor/range))
			rgb=(${color_map[c]//,/ })
			r=${rgb[0]}
			g=${rgb[1]}
			b=${rgb[2]}
			echo -n -e "\033[48;2;${r};${g};${b}m    "
		done; echo -e "\033[0m"; done
	fi
done
