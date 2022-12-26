// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#define STR_LENGTH 255


// global variables

char* f_read =  "data.txt";	// name of file to read
char* f_write =  "tmp.txt";	// name of file to write
char sep = ' ';		// separator character


// busy wait function

int msleep(int millisec) {
	usleep(millisec * 1000L);
	return(0);
}


// vector dot product function

double vdot(double ax, double ay, double az, double bx, double by, double bz) {
	double ret = ax * bx + ay * by + az * bz;
	return ret;
}


// vector absolute value function

double vabs(double x, double y, double z) {
	double ret = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
	return ret;
}


// unit vector function

double vunit(double x, double y, double z, int select) {
	double ret;
	if (select == 0)
		ret = x / vabs(x, y, z);
	if (select == 1)
		ret = y / vabs(x, y, z);
	if (select == 2)
		ret = z / vabs(x, y, z);
	return ret;
}



// checking function if there is a string

int strchk(const char* s) {
	if (s == NULL)
		return 0;
	if (strlen(s) == 0)
		return 0;
	return 1;
}


// enter deleting function

void delete_enter(char* s) {
	if (strchk(s) && s[strlen(s) - 1] == '\n') {
		s[strlen(s) - 1] = 0;
	}
}


// read data function

char* read_data(const char* s, int* begin) {
	if ((!strchk(s)) || *begin < 0) {
		return NULL;
	}
	
	char* fr = (char*)malloc(STR_LENGTH + 1);
	strcpy(fr, s + *begin);
	char* to = strchr(fr, sep);
	
	if (to == NULL) {
		*begin = -1;
		return fr;
	}
	
	int length = to - fr;
	*begin = *begin + length + 1;
	char* ret = (char*)malloc(length + 1);
	memset(ret, 0, length + 1);
	strncpy(ret, fr, length);
	
	free(fr);
	return ret;
}


// count lines function

int count_lines() {
	FILE *fq;
	char c;
	int count = 0;

	fq = fopen(f_write, "r");
	for (c = getc(fq); c != EOF; c = getc(fq))
		if (c == '\n')
			count = count + 1;

	return(count);
	fclose(fq);
}


// main function

int main() {

	FILE *fp;
	fp = fopen(f_read, "r");
	
	FILE *fq;
	fq = fopen(f_write, "w");
	
	char* s = (char*)malloc(STR_LENGTH + 1);
	char* timestamp = (char*)malloc(STR_LENGTH + 1);
	char* ECI_x = (char*)malloc(STR_LENGTH + 1);
	char* ECI_y = (char*)malloc(STR_LENGTH + 1);
	char* ECI_z = (char*)malloc(STR_LENGTH + 1);
	char* ECI_sun_x = (char*)malloc(STR_LENGTH + 1);
	char* ECI_sun_y = (char*)malloc(STR_LENGTH + 1);
	char* ECI_sun_z = (char*)malloc(STR_LENGTH + 1);
	int m;
	char* data;
	double v1x, v1y, v1z, v2x, v2y, v2z, angle, v1n, v2n, dotp;
	double u1x, u1y, u1z, u2x, u2y, u2z;

	printf("time\t\tangle\n");

	while (!feof(fp)) {
		memset(s, 0, STR_LENGTH + 1);
		fgets(s, STR_LENGTH, fp);
		if ((s[0] != sep) && strchk(s)) {
			delete_enter(s);
			m = 0;
			
			// read part
			
			data = read_data(s, &m);
			strcpy(timestamp, data);
			free(data);
			printf("%s\t", timestamp);
			
			data = read_data(s, &m);
			strcpy(ECI_x, data);
			free(data);
			//printf("%s ", ECI_x);
			
			data = read_data(s, &m);
			strcpy(ECI_y, data);
			free(data);
			//printf("%s ", ECI_y);
			
			data = read_data(s, &m);
			strcpy(ECI_z, data);
			free(data);
			//printf("%s ", ECI_z);
			
			data = read_data(s, &m);
			strcpy(ECI_sun_x, data);
			free(data);
			//printf("%s ", ECI_sun_x);
			
			data = read_data(s, &m);
			strcpy(ECI_sun_y, data);
			free(data);
			//printf("%s ", ECI_sun_y);
			
			data = read_data(s, &m);
			strcpy(ECI_sun_z, data);
			free(data);
			//printf("%s ", ECI_sun_z);
			
			// calculation part
			
			v1x = atof(ECI_x);
			v1y = atof(ECI_y);
			v1z = atof(ECI_z);
			v2x = atof(ECI_sun_x);
			v2y = atof(ECI_sun_y);
			v2z = atof(ECI_sun_z);
			
			dotp = vdot(v1x, v1y, v1z, v2x, v2y, v2z);
			v1n = vabs(v1x, v1y, v1z);
			v2n = vabs(v2x, v2y, v2z);
			angle = acos(dotp / (v1n * v2n));
			
			printf("%lf\n", angle);
			
			u1x = vunit(v1x, v1y, v1z, 0);
			u1y = vunit(v1x, v1y, v1z, 1);
			u1z = vunit(v1x, v1y, v1z, 2);
			u2x = vunit(v2x, v2y, v2z, 0);
			u2y = vunit(v2x, v2y, v2z, 1);
			u2z = vunit(v2x, v2y, v2z, 2);
			
			// write part
			
			fprintf(fq, "0 0 0\n%lf %lf %lf\n0 0 0\n%lf %lf %lf\n", u1x, u1y, u1z, u2x, u2y, u2z);
		}
	}
	
	fclose(fp);
	fclose(fq);
	
	
	// plot part
	
	int indexmax = count_lines() - 1;
	
	FILE* fw;
	fw = popen("gnuplot", "w");
	fprintf(fw, "set xlabel 'x axis'\n");
	fprintf(fw, "set ylabel 'y axis'\n");
	fprintf(fw, "set zlabel 'z axis'\n");
	fprintf(fw, "set title 'Earth-Sun angle from the perspective of the satellite'\n");
	fprintf(fw, "set view equal xyz\n");

	while(1) {
		for (int i = 0; i < indexmax-4; i+=4) {
			fprintf(fw, "splot [-1:1] [-1:1] [-1:1] 'tmp.txt' every ::%i::%i using 1:2:3 with lines\n", i, i+4);
			fflush(fw);
			msleep(50);
		}
	}

	pclose(fw);	
	return(0);
}
