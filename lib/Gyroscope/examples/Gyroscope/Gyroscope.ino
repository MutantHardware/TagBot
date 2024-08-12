/*
 * Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 * Copyright (c) 2020 Romain JL. FETICK
 * Forked and modified by João V.M Grando
 * Modifications:
 * - Changed the `constructor` and `begin` method to work with different I2C implementations.
 * - Renamed some methods for clarity and consistency.
*/

// Including Gyroscope Library
#include "Wire.h"
#include <Gyroscope.h>

// Variables and Constants
unsigned long timer = 0;

// I2C pinout
#define I2C_SDA 23 // GPIO23
#define I2C_SCL 22 // GPIO22

// Creating Wire Object
TwoWire I2C = TwoWire(0);

// Creating Gyro Object
Gyroscope Gyro;
 
void setup() {
  // Initializing Serial 
  Serial.begin(115200);
  
  // Initializing I2C communication
  I2C.begin(I2C_SDA, I2C_SCL, 400000);
  byte status = Gyro.begin(I2C,0x68);
  
  Serial.print(F("Gyroscope status: "));
  Serial.println(status);
  
  // Stay in the loop if Gyro don't work
  while(status!=0){

  } 
  
  // Calculate Offsets
  Serial.println(F("Calculating offsets, do not move Gyroscope"));
  delay(1000);
  Gyro.calcOffsets(true,true); 
  Gyro.setFilterGyroCoef(0.95);
  Serial.println("Done!\n");
}

void loop() {
  // Update Gyro 
  Gyro.update();
  
  // Print Data Every Second
  if(millis() - timer > 1000) { 

    // Print Gyro Results
    Serial.print(F("Temperature: "));
    Serial.println(Gyro.Temperature());

    Serial.print(F("Accelero X: "));
    Serial.print(Gyro.AccX());
    Serial.print("\tY: ");
    Serial.print(Gyro.AccY());
    Serial.print("\tZ: ");
    Serial.println(Gyro.AccZ());
  
    Serial.print(F("Gyro X: "));
    Serial.print(Gyro.GyroX());
    Serial.print("\tY: ");
    Serial.print(Gyro.GyroY());
    Serial.print("\tZ: ");
    Serial.println(Gyro.GyroZ());
  
    Serial.print(F("Acc Angle X: "));
    Serial.print(Gyro.AccAngleX());
    Serial.print("\tY: ");
    Serial.println(Gyro.AccAngleY());
    
    Serial.print(F("Angle X: "));
    Serial.print(Gyro.AngleX());
    Serial.print("\tY: ");
    Serial.print(Gyro.AngleY());
    Serial.print("\tZ: ");
    Serial.println(Gyro.AngleZ());
 
    // Restart Millis
    timer = millis();
  }

}