#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

char* filename =  "data.txt";
int count;

int msleep(int millisec) {
	usleep(millisec * 1000L);
	return(0);
}

int count_lines() {
	FILE *fp;
	char c;
	count = 0;

	fp = fopen(filename, "r");
	for (c = getc(fp); c != EOF; c = getc(fp))
		if (c == '\n')
			count = count + 1;

	return(count);
	fclose(fp);
}

int main(int argc, char *argv[]) {

	int indexmax = count_lines() - 1;
	printf("number of lines\t%i\n", indexmax);

	FILE *fw;

	fw = popen("gnuplot", "w");
	fprintf(fw, "set xlabel 'x axis'\n");
	fprintf(fw, "set ylabel 'y axis'\n");
	fprintf(fw, "set zlabel 'z axis'\n");
	fprintf(fw, "set title 'Satellite orbit'\n");
	fprintf(fw, "set view equal xyz\n");

	while(1) {
		for (int i = 0; i < indexmax; i++) {
			fprintf(fw, "splot [-8000:8000] [-8000:8000] [-8000:8000] 'data.txt' every ::%i::%i using 2:3:4\n", i, i);
			fflush(fw);
			msleep(50);
		}
	}

	pclose(fw);
	return(0);
}
