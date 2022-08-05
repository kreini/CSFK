// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// random number function

float rnd(int order) {
	float ret = (float)(rand()%((int)pow(10, order) + 1)) / pow(10, order);
	return(ret);
}


// print vector function

void printv(float v[4]){
	printf("%f\t%f\t%f\t%f\n", v[0], v[1], v[2], v[3]);
}


// transform to RA-DEC-ROLL

int main() {
	float u1 = rnd(3);
	float u2 = rnd(3);
	float u3 = rnd(3);
	
	float qr = sqrt(1-u1)*sin(2*M_PI*u2);
	float qi = sqrt(1-u1)*cos(2*M_PI*u2);
	float qj = sqrt(u1)*sin(2*M_PI*u3);
	float qk = sqrt(u1)*cos(2*M_PI*u3);
	
	float h[4] = {qr, qi, qj, qk};
	
	printv(h);
	
	return(0);
}
