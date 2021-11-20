#!/bin/bash

echo 256 | \
awk \
 '{	N=$1;
	for ( i=0; i<N; i++ )
	 {	for ( j=0; j<N; j++ )
		 {	printf("%3d %3d 0\n",i,j);
			printf("%3d %3d 1\n",i,j);
		 }
	 }
 }' | \
./awc | \
awk -v line=0 \
 '{	a=$1;
	b=$2;
	c=$3;
	sum=$4;
	carry=$5;
	v1=$6;
	v2=$7;
	if ( v1 != v2 )
	 {	print a,b,c;
	 }
 }'


