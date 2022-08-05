// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>


// global variables

int c = 10000;			// number of loops
float v[3] = {1, 7, 8};	// vector
char* filename =  "tmp.txt";	// name of file


// busy wait function

int msleep(int millisec) {
	usleep(millisec * 1000L);
	return(0);
}


// random number function

float rnd(int order) {
	float ret = (float)(rand()%((int)pow(10, order) + 1)) / pow(10, order);
	return(ret);
}


// main function

int main() {

	// data section

	FILE *fp;
	fp = fopen(filename, "w");

	for(int i = 0; i < c; i++) {
		
		float tmp[3];
		
		float u1 = rnd(3);
		float u2 = rnd(3);
		float u3 = rnd(3);
		
		float qr = sqrt(1-u1)*sin(2*M_PI*u2);
		float qi = sqrt(1-u1)*cos(2*M_PI*u2);
		float qj = sqrt(u1)*sin(2*M_PI*u3);
		float qk = sqrt(u1)*cos(2*M_PI*u3);
		
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
		
		tmp[0] = fR[0][0]*v[0] + fR[0][1]*v[1] + fR[0][2]*v[2];
		tmp[1] = fR[1][0]*v[0] + fR[1][1]*v[1] + fR[1][2]*v[2];
		tmp[2] = fR[2][0]*v[0] + fR[2][1]*v[1] + fR[2][2]*v[2];
		
		fprintf(fp, "%f %f %f\n", tmp[0], tmp[1], tmp[2]);
	}
	fclose(fp);
	
	
	// gnuplot section
	
	FILE *fw;

	fw = popen("gnuplot", "w");
	fprintf(fw, "set xlabel 'x axis'\n");
	fprintf(fw, "set ylabel 'y axis'\n");
	fprintf(fw, "set zlabel 'z axis'\n");
	fprintf(fw, "set title 'Quaternion homogenity'\n");
	fprintf(fw, "set view equal xyz\n");

	while(1) {
		fprintf(fw, "splot 'tmp.txt' with points\n");
		fflush(fw);
		msleep(50);
	}

	pclose(fw);

	
	return(0);
}
