// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// rectascension, declination and roll values

float ra = 0.34;
float dec = 0.19;
float roll = 0.78;


// print matrix function

void printm(float m[3][3]){
	for (int i = 0; i < 3; i++)
		printf("%f\t%f\t%f\n", m[i][0], m[i][1], m[i][2]);
}


// transform to SO(3) matrix

int main() {

	float xx = -cos(ra) * sin(dec) * cos(roll) + sin(ra) * sin(roll);
	float xy = cos(ra) * sin(dec) * sin(roll) + sin(ra) * cos(roll);
	float xz = cos(ra) * cos(dec);
	float yx = -sin(ra) * sin(dec) * cos(roll) - cos(ra) * sin(roll);
	float yy = sin(ra) * sin(dec) * sin(roll) - cos(ra) * cos(roll);
	float yz = sin(ra) * cos(dec);
	float zx = cos(dec) * cos(roll);
	float zy = -cos(dec) * sin(roll);
	float zz = sin(dec);

	float fR[3][3] = {
		{xx, xy, xz},
		{yx, yy, yz},
		{zx, zy, zz}
	};

	printm(fR);
	return(0);
}
