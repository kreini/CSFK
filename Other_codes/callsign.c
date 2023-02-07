/* Morse code call sign transmit practising code */
/* Kreinicker Gabor */


/* includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/* global variables */
int a, b, c, d, e, f, g;
char* prefix[] = {"CT", "DL", "EA", "EI", "ER", "ES", "EU", "EV", "EW", "F", "G", "HA", "HG", "HB", "HB0", "HV", "I", "IS", "LA", "LX", "LY", "UP", "LZ", "OE", "OH", "OK", "OM", "ON", "OZ", "PA", "S5", "SM", "SP", "SV", "T7", "T9", "TF", "TK", "UA1", "UR", "YL", "YO", "YU", "YT", "YZ", "Z3", "ZA", "ZB2", "3A", "9A", "9H"};

int number_of_generating = 10;	// the number of callsigns to be generated
int mode = 2;				// 1 - only 3 characters long suffix callsigns
					// 2 - only 3 characters long suffix hungarian callsigns

/* random generating parameter function */
void rnd() {
	a = rand()%3 + 2;	// the number of characters in suffix
	b = rand()%10;		// the number in prefix
	c = rand()%51;		// prefix
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

int main() {

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
