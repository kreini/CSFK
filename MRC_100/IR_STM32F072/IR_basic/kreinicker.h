/****************************************************************/
/**** Kreinicker Gábor ********* Terminál függvénygyűjtemény ****/
/****************************************************************/


/*
Character comparing functions
*/


// led

int T_led(char* buff) {
	if (buff[0] == 'l' && buff[1] == 'e' && buff[2] == 'd' && buff[3] == ':')
		return 1;
	else
		return 0;
}


// runtime

int T_run(char* buff, int ci) {
	char* chi;
	sprintf(chi, "%i");
	char* answ1 = "The code is running for: " + chi + " seconds\n";
	if (strcmp(buff, "runtime\n") == 0)
		fwrite(answ, 1, strlen(answ), stdout);
}


