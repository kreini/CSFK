// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// quaternion values

float qr = 0.46207942930874324;
float qi = -0.6078499406794751;
float qj = -0.0825619160914830;
float qk = -0.6404565407870916;


// print matrix function

void printm(float m[3][3]){
	for (int i = 0; i < 3; i++)
		printf("%f\t%f\t%f\n", m[i][0], m[i][1], m[i][2]);
}


// transform to SO(3) matrix

int main() {

	float xx = 1 - 2*(pow(qj, 2) + pow(qk, 2));
	float xy = 2*(qi*qj - qk*qr);
	float xz = 2*(qi*qk + qj*qr);
	float yx = 2*(qi*qj + qk*qr);
	float yy = 1 - 2*(pow(qi, 2) + pow(qk, 2));
	float yz = 2*(qj*qk - qi*qr);
	float zx = 2*(qi*qk - qj*qr);
	float zy = 2*(qj*qk+qi*qr);
	float zz = 1 - 2*(pow(qi, 2) + pow(qj, 2));

	float fR[3][3] = {
		{xx, xy, xz},
		{yx, yy, yz},
		{zx, zy, zz}
	};

	printm(fR);
	return(0);
}
