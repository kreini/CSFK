#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int msleep(int millisec) {
	usleep(millisec * 1000L);
	return(0);
}

int main(int argc, char *argv[]) {
	FILE *fw;
	double phase;

	fw = popen("gnuplot", "w");
	fprintf(fw, "set term x11\n");
	fprintf(fw, "set samples 1000\n");
	phase = 0.0;

	while (1) {
		fprintf(fw, "phase=%g\n", phase);
		fprintf(fw, "plot [0:20] sin(x + phase), sin(x + 2*pi/3 + phase), sin(x - 2*pi/3 + phase)\n");
		fflush(fw);
		phase += 0.05;
		msleep(100);
	}

	pclose(fw);
	return(0);
}
