// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// rectascension, declination and roll values

float ra = 0.34;
float dec = 0.19;
float roll = 0.78;


// print vector function

void printv(float v[4]){
	printf("%f\t%f\t%f\t%f\n", v[0], v[1], v[2], v[3]);
}


// transform to quaternion

int main() {

	float qr = (sqrt(1+(-cos(ra)*sin(dec)*cos(roll)+sin(ra)*sin(roll)) + (sin(ra)*sin(dec)*sin(roll)-cos(ra)*cos(roll))+sin(dec)))/2;
	float qi = (-cos(dec)*sin(roll) - sin(ra)*cos(dec))/(2*sqrt(1+(-cos(ra)*sin(dec)*cos(roll)+sin(ra)*sin(roll))+(sin(ra)*sin(dec)*sin(roll)-cos(ra)*cos(roll))+sin(dec)));
	float qj = (cos(ra)*cos(dec) - cos(dec)*cos(roll))/(2*sqrt(1+(-cos(ra)*sin(dec)*cos(roll)+sin(ra)*sin(roll))+(sin(ra)*sin(dec)*sin(roll)-cos(ra)*cos(roll))+sin(dec)));
	float qk = (-sin(ra)*sin(dec)*cos(roll)-cos(ra)*sin(roll)-(cos(ra)*sin(dec)*sin(roll)+sin(ra)*cos(roll)))/(2*sqrt(1+(-cos(ra)*sin(dec)*cos(roll)+sin(ra)*sin(roll))+(sin(ra)*sin(dec)*sin(roll)-cos(ra)*cos(roll))+sin(dec)));
	
	float fQ[4] = {qr, qi, qj, qk};

	printv(fQ);
	return(0);
}
