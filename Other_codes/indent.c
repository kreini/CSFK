/* Code indent between Gabor's and Andris' code */
/* Kreinicker Gabor */


/* includes */
#include <stdio.h>


/* main function */
int main(int argc, char** argv) {

	FILE *input_file, *output_file;
	char* input_filename;
	char* output_filename;
	char c, c_pre;
	int space_cnt = 0;
	int is_comment = 0;
	

	/* argument configuration */
	if (argc < 3) {
		printf("too few arguments\n");
		printf("%s <input> <output>\n", argv[0]);
		return 0;
	}
	if (argc > 3) {
		printf("too many arguments\n");
		printf("%s <input> <output>\n", argv[0]);
		return 0;
	}
	if (argc == 3) {
		input_filename = argv[1];
		output_filename = argv[2];
	}
	
	

	/* Open the input file in read mode */
	input_file = fopen(input_filename, "r");
	if (input_file == NULL) {
		printf("failed to open the input file.n");
		return 1;
	}

	/* Open the output file in write mode */
	output_file = fopen(output_filename, "w");
	if (output_file == NULL) {
		printf("failed to create the output file\n");
		fclose(input_file);
		return 1;
	}

	/* Process eac character in the input file */
	while ((c = fgetc(input_file)) != EOF) {
		if (c == '\t') {
			fputc(' ', output_file);
			space_cnt++;
		}
		else if (c == '{' && c_pre == ' ') {
			fputc('\n', output_file);
			for (int i = 0; i < space_cnt; i++) {
				fputc(' ', output_file);
			}
			fputc(c, output_file);
		}
		else if (c == '\n') {	
			space_cnt = 0;
			if (is_comment == 1) {
				fputc(' ', output_file);
				fputc('*', output_file);
				fputc('/', output_file);
			}
			fputc(c, output_file);
			is_comment = 0;			
		}
		else if (c == '/' && c_pre == '/') {
			fputc('*', output_file);
			is_comment = 1;
		}
		else {
			fputc(c, output_file);
		}
		c_pre = c;
	}
	
	fclose(input_file);
	fclose(output_file);

	printf("done\n");
	return 0;
}
