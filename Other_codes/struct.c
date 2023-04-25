/* Code for representating structure parameter functions */
/* if we need to change a function's parameters, it is the easiest way */
/* Gabor Kreinicker, Andras Pal */



#include <stdio.h>
#include <stdint.h>



/* structure for a, b, c variables */
struct abc {
	int a;
	int b;
	int c;
};



/* basic function for calculating r */
int function_basic(int a, int b, int c) {
	int r;
	r = a*b + c;
	return(r);
}

/* same function but with structure */
int function_struct(struct abc* f) {
	int r;
	r = (f->a)*(f->b) + (f->c);
	return(r);
}



/* main function */
int main() {
	
	/* variables for basic function */
	int a = 2;
	int b = 4;
	int c = 1;
	
	/* variables for structure basef function */
	struct abc f;
	f.a = 2;
	f.b = 4;
	f.c = 1;
	
	printf("basic:\t%i\n", function_basic(a, b, c));
	printf("struct:\t%i\n", function_struct(&f));
	
	/*printf("kincso\t");*/
	return(0);
}
