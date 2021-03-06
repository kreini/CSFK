#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>
#include <errno.h>
#include <sys/time.h>

#include "predict.h"

int fprint_usage(FILE *fw, char *argv0) {
	fprintf(fw,"Usage:\t%s [-h|--help]\n", argv0);
	fprintf(fw,"\t[-l|--location <latitude>,<longitude>[,<elevation>]]\n");
	fprintf(fw,"\t[-t|--time <UNIX_time>]\n");
	fprintf(fw,"\t[-f|--tle-file <file>] [-{0,1,2}|--tle-{0,1,2} \"<TLE>\"]\n");
	return(0);
}

int is_limiter(int chr) {
	if (isspace(chr) || chr == ',')
		return(1);
	else
		return(0);
}

int remove_newlines(char *in) {
	char *out;
	for (out = in; *in; in++) {
		if (!(*in == 10 || *in == 13))
			*out++=*in;
	}
	*out = 0;
	return(0);
}

int main(int argc, char *argv[]) {
	int i, timestamp, is_terse;
	double lon, lat, alt;
	char *timestamp_list, timestamp_curr[32];

	char *tle0 = "GRBAlpha";
 	char *tle1 = "1 47959U 21022AD  21147.26438853  .00000679  00000-0  51056-4 0  9998";
	char *tle2 = "2 47959  97.5620  49.8527 0023120  28.2565  89.7705 15.05674208  8682";
	char *tle_file;
	char tle_data0[80];
	char tle_data1[80];
	char tle_data2[80];

	struct observation obs;
	int loop;

	FILE *fw;

	lat = 47.5;
	lon = 19.0;
	alt = 100.0;

	is_terse = 0;
	timestamp = 0;
	timestamp_list = NULL;

	tle_file = NULL;

	for (i = 1; i < argc; i++) {
		int w;

		if (strcmp(argv[i],"-h") == 0 || strcmp(argv[i],"--help") == 0) {
			fprint_usage(stdout, argv[0]);
			return(0);
		}
		else if ((strcmp(argv[i],"-f")==0 || strcmp(argv[i],"--tle-file")==0) && i<argc-1) {
			i++;
			tle_file=argv[i];
		}
		else if ((strcmp(argv[i],"-0")==0 || strcmp(argv[i],"--tle0")==0 || strcmp(argv[i],"--tle-0")==0) && i<argc-1) {
			i++;
			tle0=argv[i];
		}
		else if ((strcmp(argv[i],"-1")==0 || strcmp(argv[i],"--tle1")==0 || strcmp(argv[i],"--tle-1")==0 ) && i<argc-1) {
			i++;
			tle1=argv[i];
		}
		else if ((strcmp(argv[i],"-2")==0 || strcmp(argv[i],"--tle2")==0 || strcmp(argv[i],"--tle-2")==0) && i<argc-1) {
			i++;
			tle2=argv[i];
		}
		else if ((strcmp(argv[i],"-t")==0 || strcmp(argv[i],"--time")==0) && i<argc-1 && sscanf(argv[i+1],"%d",&w)==1) {
			i++;
			timestamp=w;
			timestamp_list=argv[i];
		}
		else if ((strcmp(argv[i],"-i")==0 || strcmp(argv[i],"--instance")==0) && i<argc-1 && sscanf(argv[i+1],"%d",&w)==1) {
			i++;
			timestamp=w;
		}
		else if ((strcmp(argv[i],"-l")==0 || strcmp(argv[i],"--location")==0) && i<argc-1 && 2<=sscanf(argv[i+1],"%lg,%lg,%lg",&lat,&lon,&alt))
			i++;
		else if (strcmp(argv[i],"-e")==0 || strcmp(argv[i],"--terse")==0)
			is_terse=1;
		else {
			fprintf(stderr,"%s: error: invalid command line argument near '%s'.\n",argv[0],argv[i]);
			return(1);
		}
	}

	if (timestamp <= 0) {
		struct timeval tv;
		gettimeofday(&tv, NULL);
		sprintf(timestamp_curr,"%d.%.6d", (int)tv.tv_sec, (int)tv.tv_usec);
		timestamp_list = timestamp_curr;
	}

	if (!predict_is_valid(tle1, tle2)) {
 		fprintf(stderr,"%s: error: invalid TLE\n",argv[0]);
		return(1);
	}

	if (tle_file != NULL) {
		FILE *fr;
		if (strcmp(tle_file,"-") ==0)
			fr = stdin;
		else if ((fr = fopen(tle_file,"rb")) == NULL) {
			fprintf(stderr,"%s: error: unable to open TLE file '%s': %s.\n",argv[0],tle_file,strerror(errno));
			return(1);
		}
		fgets(tle_data0, 80, fr);
		fgets(tle_data1, 80, fr);
		fgets(tle_data2, 80, fr);
		tle0 = tle_data0;
		tle1 = tle_data1;
		tle2 = tle_data2;
		remove_newlines(tle0);
		remove_newlines(tle1);
		remove_newlines(tle2);
		fclose(fr);
	}

	fw = stdout;/*fopen("out.txt","w");*/
	loop = 50;
	/*for (int j = 0; j < loop; j++) {*/
		for (i = 0; i < 20; i++) {
			while (timestamp_list) {
				double timestamp;

				sscanf(timestamp_list,"%lg",&timestamp);
				if ((i = predict_observation(tle0,tle1,tle2,timestamp,lat,lon,alt,&obs))) {
					fprintf(stderr,"%s: error: invalid TLE (error code=%d)\n",argv[0],i);
					return(1);
				}
				if (is_terse)
					fprintf(fw,"%.3f %g %g %g %g %g %g\n",timestamp,obs.doppler,obs.elevation,obs.azimuth,obs.longitude,obs.latitude,obs.altitude);
				else {
				 	fprintf(fw, "# %.3f\n", timestamp);
					fprintf(fw, "doppler=%g\n", obs.doppler);
					fprintf(fw, "elevation=%g\n", obs.elevation);
					fprintf(fw, "azimuth=%g\n", obs.azimuth);
			  		fprintf(fw, "longitude=%g\n", obs.longitude);
					fprintf(fw, "latitude=%g\n", obs.latitude);
					fprintf(fw, "altitude=%g\n", obs.altitude);
					predict_fprint_observation(fw, &obs);
					fprintf(fw, "\n\n");
				}
				while (*timestamp_list) {
					if (is_limiter(*timestamp_list))
						break;
					timestamp_list++;
				}
				if (! *timestamp_list)
					timestamp_list = NULL;
				if (timestamp_list) {
					while (is_limiter(*timestamp_list))
						timestamp_list++;
				}
			}
		}
	/*}*/
	fclose(fw);
	return(0);
}
