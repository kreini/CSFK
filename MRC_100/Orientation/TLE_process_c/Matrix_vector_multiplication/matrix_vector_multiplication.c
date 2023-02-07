// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// global variables

int v[3] = {1, 7, 8};
int m[3][3] = {
	{1, 3, 3},
	{1, 3, 4},
	{1, 4, 3}
};


// print vector function

void printv(float v[3]){
	printf("%f\t%f\t%f\n", v[0], v[1], v[2]);
}


// transform to RA-DEC-ROLL

int main() {
	float ret[3];
	
	ret[0] = m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2];
	ret[1] = m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2];
	ret[2] = m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2];
	
	printv(ret);
	
	return(0);
}
