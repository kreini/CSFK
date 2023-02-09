#include <Stepper.h>
#include <math.h>

const int stepsPerRevolution = 16;  // 64 is good

// MPU9250 IMU(Wire, 0x68);
int status;
// for your motor

                                         
// initialize the stepper library for both steppers:
Stepper small_stepper(stepsPerRevolution, 9,11,10,12);  
Stepper small_stepper2(stepsPerRevolution, 2,4,3,5); 

void setup() {

  // set the speed of the motors 
  small_stepper.setSpeed(300);    // set first stepper speed
  small_stepper2.setSpeed(300);   // set second stepper speed

  Serial.begin(9600);
}


void loop() {
 
  /*int sensorReading = analogRead(A0); // read value from joystick Y-axis

  if (sensorReading < 490) { small_stepper.step(1); }   // step left
  if (sensorReading > 540) { small_stepper.step(-1); }  // step right
 
  int sensorReading2 = analogRead(A1); // read value from joystick X-axis

  if (sensorReading2 < 490) { small_stepper2.step(-1); } // step forward
  if (sensorReading2 > 540) { small_stepper2.step(1); } // step backward
  */

  for (int i = 0; i < 250; i++) {
    small_stepper.step(-1);
    small_stepper2.step(2);
    delay(10);
  }
  for (int i = 0; i < 100; i++) {
    small_stepper2.step(2);
    delay(10);
  }
  for (int i = 0; i < 250; i++) {
    small_stepper.step(1);
    small_stepper2.step(2);
    delay(10);
  }
  delay(1000);
  small_stepper2.step(-1200);
  delay(1000);
  
  
 }
