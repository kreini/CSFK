// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// SO(3) matrix

float R[3][3] = {
	{-0.04435591360376488, 0.34926122321939446, 0.9359749734280653},
	{-0.22642312284119023, -0.9160309672169911, 0.33108886502319457},
	{0.9730185621925301, -0.19724062729264522, 0.1197122072889193}
};


// print vector function

void printv(float v[4]){
	printf("%f\t%f\t%f\t%f\n", v[0], v[1], v[2], v[3]);
}


// transform to quaternion

int main() {

	float qr = (sqrt(1+R[0][0]+R[1][1]+R[2][2]))/2;
	float qi = (R[2][1] - R[1][2])/(2*sqrt(1+R[0][0]+R[1][1]+R[2][2]));
	float qj = (R[0][2] - R[2][0])/(2*sqrt(1+R[0][0]+R[1][1]+R[2][2]));
	float qk = (R[1][0] - R[0][1])/(2*sqrt(1+R[0][0]+R[1][1]+R[2][2]));
	
	float fQ[4] = {qr, qi, qj, qk};

	printv(fQ);
	return(0);
}
