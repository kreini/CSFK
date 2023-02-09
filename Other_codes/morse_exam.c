/* Morse exam test sheet generator */
/* Kreinicker Gabor */


/* includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* global variables */
char* filename =  "generated.txt";
int c, tmp, rndi;
int ind = 0;
int kernel[26];
int number_of_generating = 1;	// the number of callsigns to be generated


/* kernel update */
void kernel_update() {
	for (int j = 0; j < 1000; j++) {
		for (int i = 0; i < 26; i++) {
			tmp = kernel[i];
			rndi = rand()%26;
			kernel[i] = kernel[rndi];
			kernel[rndi] = tmp;
		}
	}
}


/* main function */
int main(int argc, char** argv) {

	/* argument configuration */
	if (argc == 1) {
		printf("%s <number of test sheets>\n", argv[0]);
		return 0;
	}
	if (argc > 1)
		number_of_generating = atoi(argv[1]);
	if (argc > 2)
		printf("Too many input arguments!\n");

	/* init random */
	time_t t = time(0);
	srand((unsigned int) t);

	/* data section */
	FILE *fp;
	fp = fopen(filename, "w");
	
	/* kernel init */
	for (int i = 0; i < 26; i++)
		kernel[i] = i + 65;
	
	/* generating */
	for (int k = 0; k < number_of_generating; k++) {
		for (int i = 0; i < 18; i++) {		
			kernel_update();
			for (int j = 0; j < 5; j++) {
				c = kernel[j];
				fprintf(fp, "%c", c);
			}
			if (ind == 4) {
				fprintf(fp, "\n");
				ind = 0;
			}
			else {
				fprintf(fp, "\t");
				ind++;
			}
		}
		fprintf(fp, "\n\n");
		ind = 0;
	}
	fclose(fp);
	
	
	// printf("kincso\n");
	return 0;
}
