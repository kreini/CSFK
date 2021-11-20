/*************************************/
/*** Arduino 3 channel analog read ***/
/*************************************/
/********* Kreinicker Gábor **********/
/*************************************/



int a1 = A1;
int a2 = A2;
int a3 = A3;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print(analogRead(a1));
  Serial.print("\t");
  Serial.print(analogRead(a2));
  Serial.print("\t");
  Serial.println(analogRead(a3));
}
