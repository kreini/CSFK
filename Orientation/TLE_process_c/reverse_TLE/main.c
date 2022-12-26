// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>


// global variables

typedef double vector[3];
typedef double matrix[3][3];
typedef double quaternion[4];

vector r = {0, 0, 0};
vector s = {0, 0, 0};
quaternion H = {0, 0, 0, 0};
matrix M = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
matrix R = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// TLE 2022.09.19.
vector ECI = {-1171.695379, 1362.256544, 6679.652156};
vector ECI_sun = {-150009328.380701, 8276042.514482, 3587901.348798};



// print vector

void printv(vector r) {
	printf("%lf\t%lf\t%lf\n", r[0], r[1], r[2]);
}


// print matrix

void printm(matrix R) {
	printf("%lf\t%lf\t%lf\n%lf\t%lf\t%lf\n%lf\t%lf\t%lf\n", R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]);
}


// print quaternion

void printh(quaternion H) {
	printf("%lf\t%lf\t%lf\t%lf\n", H[0], H[1], H[2], H[3]);
}


// copy vector

void vector_cpy(vector r, vector a){
	for (int i = 0; i < 3; i++)
		r[i] = a[i];
}


// random quaternion function

double drand(void) {	
	double ret = (double)(rand()%(1000 + 1)) / 1000;
	return ret;
}

void random_quaternion() {

	time_t t = time(0);
	srand((unsigned int) 0); // t

	double u1 = drand();
	double u2 = drand();
	double u3 = drand();
	
	H[0] = sqrt(1-u1)*sin(2*M_PI*u2);
	H[1] = sqrt(1-u1)*cos(2*M_PI*u2);
	H[2] = sqrt(u1)*sin(2*M_PI*u3);
	H[3] = sqrt(u1)*cos(2*M_PI*u3);
}


// quaternion to SO(3) matrix

void quaternion_to_so3(quaternion H) {
	M[0][0] = 1 - 2*(pow(H[2], 2) + pow(H[3], 2));
	M[0][1] = 2*(H[1]*H[2] - H[3]*H[0]);
	M[0][2] = 2*(H[1]*H[3] + H[2]*H[0]);
	M[1][0] = 2*(H[1]*H[2] + H[3]*H[0]);
	M[1][1] = 1 - 2*(pow(H[1], 2) + pow(H[3], 2));
	M[1][2] = 2*(H[2]*H[3] - H[1]*H[0]);
	M[2][0] = 2*(H[1]*H[3] - H[2]*H[0]);
	M[2][1] = 2*(H[2]*H[3]+H[1]*H[0]);
	M[2][2] = 1 - 2*(pow(H[1], 2) + pow(H[2], 2));
}


// matrix vector multipler

void matrix_vector_multipler(matrix A, vector b, vector r) {
	for (int i = 0; i < 3; i++)
		r[i] = A[i][0]*b[0]+A[i][1]*b[1]+A[i][2]*b[2];
}


// vector vector scalar multiplication (r=a*b)

double vector_scalar_product(vector a, vector b) {
	double r;
	r = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
	return r;
}


// quaternion scalar product

double quaternion_scalar_product(quaternion a, quaternion b) {
	double r;
	r = a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3];
	return r;
}


// vector normalization (a->|a|)

void vector_normalize(vector a) {
	double v;
	v = sqrt(vector_scalar_product(a, a));
	if (v > 0.0) {
		a[0] = a[0] / v;
		a[1] = a[1] / v;
		a[2] = a[2] / v;
	}
}


// quaternion normalization (a->|a|)

void quaternion_normalize(quaternion a) {
	double v;
	v = sqrt(quaternion_scalar_product(a, a));
	if (v > 0.0) {
		a[0] = a[0] / v;
		a[1] = a[1] / v;
		a[2] = a[2] / v;
		a[3] = a[3] / v;
	}
}


// 

void matrix_transposition(matrix A) {
	double w;
	w = A[0][1]; A[0][1] = A[1][0]; A[1][0] = w;
	w = A[0][2]; A[0][2] = A[2][0]; A[2][0] = w;
	w = A[1][2]; A[1][2] = A[2][1]; A[2][1] = w;
	
}


// vector vector cross product (r=axb)

void cross_product(vector r, vector a, vector b) {
	for (int i = 0; i < 3; i++) {
		r[i] = a[(i+1)%3]*b[((i+2)%3)] - a[((i+2)%3)]*b[(i+1)%3];
	}
}


// matrix matrix multiplication (R=A*B)

void matrix_multipler(matrix A, matrix B, matrix R) {
	/*
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			R[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j];
	}
	*/
	
	for (int i = 0; i < 3; i++) {
		R[i][0] = A[i][0]*B[0][0] + A[i][1]*B[1][0] + A[i][2]*B[2][0];
		R[i][1] = A[i][0]*B[0][1] + A[i][1]*B[1][1] + A[i][2]*B[2][1];
		R[i][2] = A[i][0]*B[0][2] + A[i][1]*B[1][2] + A[i][2]*B[2][2];
	}		
}


// main function

int main() {

	matrix es, ej;
	matrix N;
	
	random_quaternion();
	//H[0] = 1;
	//H[3] = 0.01;
	printf("Random quaternion\n");
	quaternion_normalize(H);
	printh(H);
	
	quaternion_to_so3(H);
	printf("Random SO(3) matrix\n");
	printm(M);
	
	matrix_transposition(M);
	
	matrix_vector_multipler(M, ECI, r);
	printf("Vector in J2000\n");
	printv(ECI);
	printf("Vector in satellite system\n");
	printv(r);
	
	matrix_vector_multipler(M, ECI_sun, s);
	printf("Vector in J2000\n");
	printv(ECI_sun);
	printf("Vector in satellite system\n");
	printv(s);
	
	/*________________________*/
	
	vector_cpy(es[0], r);
	vector_normalize(es[0]);
	cross_product(es[1], s, es[0]);
	vector_normalize(es[1]);
	cross_product(es[2], es[0], es[1]);
	vector_normalize(es[2]);
	
	vector_cpy(ej[0], ECI);
	vector_normalize(ej[0]);
	cross_product(ej[1], ECI_sun, ej[0]);
	vector_normalize(ej[1]);
	cross_product(ej[2], ej[0], ej[1]);
	vector_normalize(ej[2]);
	
	matrix_transposition(es);
	matrix_multipler(es, ej, N);
	matrix_transposition(N);
	printm(N);
	
	return(0);
}
