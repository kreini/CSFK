#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int msleep(int millisec) {
	usleep(millisec * 1000L);
	return(0);
}

int main(int argc, char *argv[]) {
	FILE *fw;
	double c;

	fw = popen("gnuplot", "w");
	fprintf(fw, "set isosamples 50\n");
	fprintf(fw, "set hidden3d\n");
	fprintf(fw, "set xlabel 'x axis'\n");
	fprintf(fw, "set ylabel 'y axis'\n");
	fprintf(fw, "set zlabel 'z axis'\n");
	fprintf(fw, "set title 'Testing gnuplot 3d plot'\n");

	c = 0.0;

	while (1) {
		fprintf(fw, "phase=%g\n", c);
		fprintf(fw, "splot [0:1.5] [0:1.5] [-0.6:0.6] exp(-(x**%f+y**%f))*cos(x/4)*sin(y)*cos(2*(x**2+y**2))\n", c, c);
		fflush(fw);
		c += 0.05;
		msleep(100);
	}

	pclose(fw);
	return(0);
}
