/* Morse code call sign transmit practising code */
/* Kreinicker Gabor */


/* includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* global variables */
int a, b, c, d, e, f, g;
char* prefix[] = {"CT", "DL", "EA", "EI", "ER", "ES", "EU", "EV", "EW", "F", "G", "HA", "HG", "HB", "HV", "I", "IS", "LA", "LX", "LY", "UP", "LZ", "OE", "OH", "OK", "OM", "ON", "OZ", "PA", "SM", "SP", "SV", "TF", "TK", "UR", "YL", "YO", "YU", "YT", "YZ", "ZA", "3A", "9A", "9H"};

int number_of_generating = 1;	// the number of callsigns to be generated
int mode = 0;			// 1 - only 3 characters long suffix callsigns
				// 2 - only 3 characters long suffix hungarian callsigns

/* random generating parameter function */
void rnd() {
	a = rand()%3 + 2;	// the number of characters in suffix
	b = rand()%10;		// the number in prefix
	c = rand()%44;		// prefix
	d = rand()%26 + 65;	// suffix (used later as chars - ascii)
	e = rand()%26 + 65;
	f = rand()%26 + 65;
	g = rand()%26 + 65;
}


/* callsign generating function */
char* tx() {
	if (mode == 1)
		a = 3;
	if (mode == 2) {
		c = rand()%2 + 11;
		a = 3;
	}
	
	if (a == 2)
		printf("%s%i%c%c\n", prefix[c], b, d, e);
	else if (a == 3)
		printf("%s%i%c%c%c\n", prefix[c], b, d, e, f);
	else if (a == 4)
		printf("%s%i%c%c%c%c\n", prefix[c], b, d, e, f, g);
	else
		printf("Something is wrong, champ!\n");	// it shouldn't be
}


/* main function */

int main(int argc, char** argv) {

	/* argument configuration */
	if (argc == 1) {
		printf("%s <mode> <number of callsigns>\n", argv[0]);
		printf("modes:\n\t0 - mixed\n\t1 - mixed with 3 long suffix\n\t2 - hungarian prefix with 3 long suffix\n");
		printf("example:\n\t%s 0 10\t - 10 mixed callsigns will be generated\n", argv[0]);
		return 0;
	}
	if (argc > 1)
		mode = atoi(argv[1]);
	if (mode < 0)
		mode = 0;
	if (mode > 2)
		mode = 2;
	if (argc > 2)
		number_of_generating = atoi(argv[2]);
	

	/* init random */
	time_t t = time(0);
	srand((unsigned int) t);
	
	printf("\n");
	for (int i = 0; i < number_of_generating; i++) {
		rnd();
		tx();
	}
	printf("\n");
		
	// printf("kincso\n");
	return 0;
}
