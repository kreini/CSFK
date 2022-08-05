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

void printv(float v[3]){
	printf("%f\t%f\t%f\n", v[0], v[1], v[2]);
}


// transform to RA-DEC-ROLL

int main() {

	float ra = atan2(R[1][2], R[0][2]);
	float dec = asin(R[2][2]);
	float roll = atan2(-R[2][1], R[2][0]);
	
	float fRDR[3] = {ra, dec, roll};

	printv(fRDR);
	return(0);
}
