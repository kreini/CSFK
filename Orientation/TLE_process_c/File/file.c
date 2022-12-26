// include libraries

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define STR_LENGTH 255


// global variables

char* f_read =  "data.txt";	// name of file to read
char* f_write =  "tmp.txt";	// name of file to write
char sep = ' ';		// separator character


// checking function if there is a string

int strchk(const char* s) {
	if (s == NULL)
		return 0;
	if (strlen(s) == 0)
		return 0;
	return 1;
}


// enter deleting function

void delete_enter(char* s) {
	if (strchk(s) && s[strlen(s) - 1] == '\n') {
		s[strlen(s) - 1] = 0;
	}
}


// read data function

char* read_data(const char* s, int* begin) {
	if ((!strchk(s)) || *begin < 0) {
		return NULL;
	}
	
	char* fr = (char*)malloc(STR_LENGTH + 1);
	strcpy(fr, s + *begin);
	char* to = strchr(fr, sep);
	
	if (to == NULL) {
		*begin = -1;
		return fr;
	}
	
	int length = to - fr;
	*begin = *begin + length + 1;
	char* ret = (char*)malloc(length + 1);
	memset(ret, 0, length + 1);
	strncpy(ret, fr, length);
	
	free(fr);
	return ret;
}


// main function

int main() {

	FILE *fp;
	fp = fopen(f_read, "r");
	
	FILE *fq;
	fq = fopen(f_write, "w");
	
	char* s = (char*)malloc(STR_LENGTH + 1);
	char* data1 = (char*)malloc(STR_LENGTH + 1);
	char* data2 = (char*)malloc(STR_LENGTH + 1);
	int m;
	char* data;
	int ret;

	while (!feof(fp)) {
		memset(s, 0, STR_LENGTH + 1);
		fgets(s, STR_LENGTH, fp);
		if ((s[0] != sep) && strchk(s)) {
			delete_enter(s);
			m = 0;
			
			// read part
			
			data = read_data(s, &m);
			strcpy(data1, data);
			free(data);
			printf("%s ", data1);
			
			data = read_data(s, &m);
			strcpy(data2, data);
			free(data);
			printf("%s ", data2);
			
			// write part
			
			ret = atoi(data2) - atoi(data1);
			printf("%i\n", ret);
			fprintf(fq, "%i\n", ret);
		}
	}
	
	fclose(fp);
	fclose(fq);
	return(0);
}
