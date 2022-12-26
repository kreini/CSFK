#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int msleep(int millisec) {
	usleep(millisec * 1000L);
	return(0);
}

int main(int argc, char *argv[]) {
	FILE *fw;

	fw = popen("gnuplot", "w");
	fprintf(fw, "set xlabel 'x axis'\n");
	fprintf(fw, "set ylabel 'y axis'\n");
	fprintf(fw, "set zlabel 'z axis'\n");
	fprintf(fw, "set title 'Satellite orbit'\n");
	fprintf(fw, "set view equal xyz\n");


	while(1) {
		fprintf(fw, "splot [-8000:8000] [-8000:8000] [-8000:8000] 'data.txt' using 2:3:4 with lines\n");
		fflush(fw);
		msleep(100);
	}

	pclose(fw);
}
