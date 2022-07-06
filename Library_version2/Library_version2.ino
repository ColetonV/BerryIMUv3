/*
  This program  reads the angles and heading from the accelerometer, gyroscope
   and compass on a BerryIMU connected to a Teensy 3.6.

  http://ozzmaker.com/

*/
#include "BerryIMU_v3.h"

BerryIMU_v3 BerryIMU;

void setup() {

  Serial.begin(115200);  // start serial for output

}

void loop() {

  BerryIMU.IMU_read();

//  Serial.print(BerryIMU.AccXraw);
//  Serial.print(",");
//  Serial.print(BerryIMU.AccYraw);
//  Serial.print(",");
//  Serial.println(BerryIMU.AccZraw);
//
//  Serial.print(BerryIMU.gyr_rateXraw);
//  Serial.print(",");
//  Serial.print(BerryIMU.gyr_rateYraw);
//  Serial.print(",");
//  Serial.println(BerryIMU.gyr_rateZraw);
//
  Serial.println(BerryIMU.alt);

  delay(10);

}
