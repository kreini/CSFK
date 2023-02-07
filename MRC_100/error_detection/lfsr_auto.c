/* linear-feedback shift register (lfsr) polynomial finding code */
/* Kreinicker Gabor */


/* includes */
#include <stdio.h>
#include <stdint.h>

/* global variables */
int x = 0b111;
int polynomial;
int polynomial_size = 24;	// the number of bits in the polynomial (try 29, ones = 2)
int ones = 4;	// the number of ones of the max polynomial
int length = 0;


/* bin and hex print function */
// source: https://stackoverflow.com/a/53850409
void printn(int x) {
	for(int j = sizeof(x)<<3; j; j--)
		putchar('0' + ((x>>(j - 1)) & 1));
	printf(" - 0x%x\n", x);
}


/* popcnt function */
int popcnt(int a) {
	int ret = 0;
	for (int l = 0; l < polynomial_size + 1; l++) {
		if (a & 1)	// checking if the last bit is equal to 1
			ret++;
		a = (a>>1);	// shift
	}
	return ret;
}


/* lfsr display only max polynomial function */
void lfsr(int a) {
	
	int a_s = a;	// store the original data
	
	for (int i = 0; i < (1<<polynomial_size); i++) {	// i = 1..2^p_s
		if (a != a_s || i == 0) {	// we do this until we get the original polynomial
			if (a & 1)	// if the last bit is one
				a = (a>>1) ^ polynomial;	// bitshift + XOR
			else
				a = a>>1;
		}
		else {
			length = i;	// get the period length
			if (length == (1<<polynomial_size) - 1) {	// if this is the max
				printn(polynomial);
				// printf("number of ones: %i\n", popcnt(polynomial));	// number of ones
			}
			break;
		}
	}
}


/* main function */
int main() {

	printf("\n%i-bit max polynomials:\nbin - hex\n", polynomial_size);
	for (int k = (1<<(polynomial_size - 1)); k < (1<<polynomial_size); k++) {	// k = 2^(n-1)..2^n
		polynomial = k;
		if (popcnt(polynomial) == ones)	// if the number of ones is equal to ones
			lfsr(x);	// do the lfsr algorythm
	}
	
	// printf("kincso\n");
	return 0;
}
