#include <Stepper.h>
#include "MPU9250.h"

const int stepsPerRevolution = 64;  // change this to fit the number of steps per revolution0

MPU9250 IMU(Wire, 0x68);
int status;
// for your motor

                                         
// initialize the stepper library for both steppers:
Stepper small_stepper(stepsPerRevolution, 9,11,10,12);  
Stepper small_stepper2(stepsPerRevolution, 2,4,3,5); 

void setup() {

  // set the speed of the motors 
  small_stepper.setSpeed(300);    // set first stepper speed
  small_stepper2.setSpeed(300);   // set second stepper speed

  Serial.begin(115200);
  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}


void loop() {
 
  int sensorReading = analogRead(A0); // read value from joystick Y-axis

  if (sensorReading < 490) { small_stepper.step(1); }   // step left
  if (sensorReading > 540) { small_stepper.step(-1); }  // step right
 
 int sensorReading2 = analogRead(A1); // read value from joystick X-axis

  if (sensorReading2 < 490) { small_stepper2.step(-1); } // step forward
  if (sensorReading2 > 540) { small_stepper2.step(1); } // step backward

  // read the sensor
  IMU.readSensor();
  // display the data
  /*Serial.print(IMU.getAccelX_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(), 6);
  Serial.print("\t");*/
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(), 6);
  //Serial.println("\t");
  //Serial.println(IMU.getTemperature_C(), 6);
  delay(5);
  
 }
