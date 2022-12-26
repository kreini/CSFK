// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>


// global variables

typedef double vector[3];
typedef double matrix[3][3];
typedef double quaternion[4];

matrix A = {{1, 3, 3}, {1, 3, 4}, {1, 4, 3}};
matrix B = {{4, 2, 0}, {0, 6, 9}, {9, 1, 1}};
matrix R = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
vector a = {5, 3, 9};
vector b = {6, 4, 3};
vector r = {0, 0, 0};

double v;
int c = 3;


// print vector

void printv(vector r) {
	printf("%lf\t%lf\t%lf\n", r[0], r[1], r[2]);
}


// print matrix

void printm(matrix R) {
	printf("%lf\t%lf\t%lf\n%lf\t%lf\t%lf\n%lf\t%lf\t%lf\n", R[0][0], R[0][1], R[0][2], R[1][0], R[1][1], R[1][2], R[2][0], R[2][1], R[2][2]);
}





// vector constant multiplication (r=ca)

void vector_constant_multipler(double c, vector a, vector r) {
	for (int i = 0; i < 3; i++)
		r[i] = a[i] * c;
}


// matrix constant multiplication (R=cA)

void matrix_constant_multipler(double c, matrix A, matrix R) {
	/*
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			R[i][j] = A[i][j] * c;
	}
	*/
	
	for (int i = 0; i < 3; i++) {
		R[i][0] = A[i][0] * c;
		R[i][1] = A[i][1] * c;
		R[i][2] = A[i][2] * c;
	}
}


// matrix vector multiplication (r=A*b)

void matrix_vector_multipler(matrix A, vector b, vector r) {
	for (int i = 0; i < 3; i++)
		r[i] = A[i][0]*b[0]+A[i][1]*b[1]+A[i][2]*b[2];
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


// vector vector scalar multiplication (r=a*b)

double scalar_product(vector a, vector b) {
	double r;
	r = a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
	return r;
}


// vector vector cross product (r=axb)

void cross_product(vector a, vector b, vector r) {
	for (int i = 0; i < 3; i++) {
		r[i] = a[(i+1)%3]*b[((i+2)%3)] - a[((i+2)%3)]*b[(i+1)%3];
	}
}


// vector normalization (a->|a|)

void vector_normalize(vector a) {
	double v;
	v = sqrt(scalar_product(a, a));
	if (v > 0.0) {
		r[0] = a[0] / v;
		r[1] = a[1] / v;
		r[2] = a[2] / v;
	}
}


// matrix determinant (det(A))

double matrix_determinant(matrix A) {
	double r;
	r = A[0][0]*(A[1][1]*A[2][2] - A[2][1]*A[1][2]) - A[1][0]*(A[0][1]*A[2][2] - A[2][1]*A[0][2]) + A[2][0]*(A[0][1]*A[1][2] - A[1][1]*A[0][2]);
	return (double)r;
}


// matrix transposition (A^T)

void matrix_transposition(matrix A, matrix R) {
	/*
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			R[i][j] = A[j][i];
	}
	*/
	
	for (int i = 0; i < 3; i++) {
		R[i][0] = A[0][i];
		R[i][1] = A[1][i];
		R[i][2] = A[2][i];
	}
}


// matrix adjugate (adj(A))

void matrix_adjugate(matrix A, matrix R) {
	/*
	// default
	R[0][0] = (A[1][1]*A[2][2] - A[2][1]*A[1][2]);
	R[0][1] = -(A[0][1]*A[2][2] - A[2][1]*A[0][2]);
	R[0][2] = (A[0][1]*A[1][2] - A[1][1]*A[0][2]);
	R[1][0] = -(A[1][0]*A[2][2] - A[2][0]*A[1][2]);
	R[1][1] = (A[0][0]*A[2][2] - A[2][0]*A[0][2]);
	R[1][2] = -(A[0][0]*A[1][2] - A[1][0]*A[0][2]);
	R[2][0] = (A[1][0]*A[2][1] - A[2][0]*A[1][1]);
	R[2][1] = -(A[0][0]*A[2][1] - A[2][0]*A[0][1]);
	R[2][2] = (A[0][0]*A[1][1] - A[1][0]*A[0][1]);
	
	// loop in the loop
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			R[j][i] = (A[(i+1)%3][(j+1)%3]*A[(i+2)%3][(j+2)%3]) - (A[(i+1)%3][(j+2)%3]*A[(i+2)%3][(j+1)%3]);
		}
	}
	*/
	
	for (int i = 0; i < 3; i++) {
		R[0][i] = (A[(i+1)%3][1]*A[(i+2)%3][2]) - (A[(i+1)%3][2]*A[(i+2)%3][1]);
		R[1][i] = (A[(i+1)%3][2]*A[(i+2)%3][0]) - (A[(i+1)%3][0]*A[(i+2)%3][2]);
		R[2][i] = (A[(i+1)%3][0]*A[(i+2)%3][1]) - (A[(i+1)%3][1]*A[(i+2)%3][0]);
	}
}


// matrix inverse (inv(A))

void matrix_inverse(matrix A, matrix R) {
	matrix_adjugate(A, R);
	matrix_constant_multipler(1/matrix_determinant(A), R, R);
	
}


// main function

int main() {

	vector_constant_multipler(c, a, r);
	printf("r=ca\n");
	printv(r);
	
	matrix_constant_multipler(c, A, R);
	printf("R=cA\n");
	printm(R);
	
	matrix_vector_multipler(A, b, r);
	printf("r=A*b\n");
	printv(r);
	
	matrix_multipler(A, B, R);
	printf("R=A*B\n");
	printm(R);
	
	v = scalar_product(a, b);
	printf("r=a*b\n%lf\n", v);
	
	cross_product(a, b, r);
	printf("r=axb\n");
	printv(r);
	
	vector_normalize(a);
	printf("a->|a|\n");
	printv(r);
	
	matrix_adjugate(A, R);
	printf("adj(A)\n");
	printm(R);
	
	v = matrix_determinant(A);
	printf("det(A)\n%lf\n", v);
	
	matrix_transposition(A, R);
	printf("A^T\n");
	printm(R);
	
	matrix_inverse(A, R);
	printf("inv(A)\n");
	printm(R);
	
	return(0);
}
