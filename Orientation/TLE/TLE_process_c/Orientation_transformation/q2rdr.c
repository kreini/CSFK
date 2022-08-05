// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// quaternion values

float qr = 0.46207942930874324;
float qi = -0.6078499406794751;
float qj = -0.0825619160914830;
float qk = -0.6404565407870916;


// print vector function

void printv(float v[3]){
	printf("%f\t%f\t%f\n", v[0], v[1], v[2]);
}


// transform to RA-DEC-ROLL

int main() {

	float ra = atan2(qj*qk - qi*qr, qi*qk + qj*qr);
	float dec = asin(pow(qr, 2) + pow(qk, 2) - pow(qi, 2) - pow(qj, 2));
	float roll = atan2(-qj*qk - qi*qr, qi*qk - qj*qr);
	
	float fRDR[3] = {ra, dec, roll};

	printv(fRDR);
	return(0);
}
