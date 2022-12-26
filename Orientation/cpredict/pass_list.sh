#!/bin/bash

CPREDICT=/home/apal/bin/cpredict

LAT=47.917
LON=19.895
NORAD=51085
T_BEG=1644213600
T_END=1644235200
TSTEP=60

SATDATA=/home/apal/work/tle/satdata

while [ -n "$1" ]; do
    case "$1" in
	-h|--help)
		echo -e	"Usage:	$0 [-h|--help]" 
		echo -e "	[-n|--norad <norad_id>]"
		echo -e "	[-l|--location <latitude>,<longitude>]"
		echo -e "	[-t|--timespan <unix_time_begin>:<unix_time_end>]"
		echo -e "	[-s|--step <stepsize/seconds>]"
		exit	0
		;;
	-n|--norad)
		NORAD="$2"
		shift
		;;
	-l|--location)
		A=(${2//,/ })
		LAT=${A[0]}
		LON=${A[1]}
		shift
		;;
	-t|--time-span)
		A=(${2//:/ })
		T_BEG=${A[0]}
		T_END=${A[1]}
		shift
		;;
	-s|--step)
		TSTEP="$2"
		shift
		;;
	*)
		echo "$0: error: invalid command line argument near '$1'." >> /dev/stderr
		exit	1
		;;
    esac
    shift
done

$CPREDICT \
	--location $LAT,$LON \
	--tle-file ${SATDATA}/$NORAD.tle \
	--time "$(seq ${T_BEG} ${TSTEP} ${T_END})" \
	--terse 2>/dev/null | \
awk -v prev=-1 -v ptime=0 -v had_aos=0 \
 '{	curr=$3;
	ctime=$1;
	if ( prev<0 && 0<curr )
	 {	dt=(ctime-ptime)*(-prev)/(curr-prev);
		aos=ptime+dt;
		had_aos=1;
	 }
	else if ( 0<prev && curr<0 && had_aos )
	 {	dt=(ctime-ptime)*(prev)/(prev-curr);
		los=ptime+dt;
		printf("%d %d\n",aos,los);
		had_aos=0;
	 }
	prev=curr;
	ptime=ctime;	
 }'

