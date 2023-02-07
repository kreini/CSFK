/* linear-feedback shift register (lfsr) test code */
/* Kreinicker Gabor */



/* includes */
#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* global variables */
int x = 0b111;
int polynomial = 0b1001101101110110;  // max 4 bit polynom: 1100, 1001
int length = 0;


/* binary print function */
// source: https://stackoverflow.com/a/53850409
void printb(int x) {
	for(int i = sizeof(x)<<3; i; i--)
		putchar('0' + ((x>>(i - 1)) & 1));
	printf("\n");
}


/* binary length function */
// source: https://stackoverflow.com/a/29389440
int number_of_bits(int bin) {
	unsigned bits, var = (bin < 0) ? -bin : bin;
	for (bits = 0; var != 0; ++bits)
		var >>= 1;
	return(bits);
}


/* lfsr display all steps function */
/*
int lfsr(int a) {
	printb(a);
	for (int i = 0; i < pow(2, number_of_bits(polynomial)); i++) {
		if (a & 1)
			a = (a>>1) ^ polynomial;
		else
			a = a>>1;
		printb(a);
	}
}
*/


/* lfsr display length only function */
void lfsr(int a) {
	int a_s = a;	// store the original data
	printb(a);
	
	for (int i = 0; i < pow(2, number_of_bits(polynomial)); i++) {	// i = 1..2^p_s
		if (a != a_s || i == 0) {	// we do this until we get the original polynomial
			if (a & 1)	// if the last bit is one
				a = (a>>1) ^ polynomial;	// bitshift + XOR
			else
				a = a>>1;
			printb(a);
		}
		else {
			length = i;	// get the period length
			printf("\npolynomial:\t");
			printb(polynomial);
			printf("length:\t\t%i\n", length);
			break;
		}
	}
}


/* main function */
int main() {

	lfsr(x);	// do the lfsr algorythm
	
	// printf("kincso\n");
	return 0;
}
