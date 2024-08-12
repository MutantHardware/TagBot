// Including Library
#include <Move.h>

// Right motor pinout
#define AIN1 04  // GPIO04
#define AIN2 02  // GPIO02
#define PWMA 15  // GPIO15

// Left motor pinout
#define BIN1 16  // GPIO16
#define BIN2 17  // GPIO17
#define PWMB 05  // GPIO05

// Creating Move object
Move Move(AIN1,AIN2,PWMA,BIN1,BIN2,PWMB);

// Velocity Definition
int DutyCycleA = 200;
int DutyCycleB = 200;

void setup() {

}

// Move robot
void loop() {
   // Move only right motor forward 
   Move.rightMotorForward();
   Move.rightMotorPWM(DutyCycleA);
   delay(1500);
   Move.rightMotorStop();
   
   // Delay to slowdown
   delay(100);

   // Move only right motor backward 
   Move.rightMotorBackward();
   Move.rightMotorPWM(DutyCycleA);
   delay(1500);
   Move.rightMotorStop();

   // Delay to slowdown
   delay(100);

   // Move only left motor forward 
   Move.leftMotorForward();
   Move.leftMotorPWM(DutyCycleB);
   delay(1500);
   Move.leftMotorStop();  

   // Delay to slowdown
   delay(100);

   // Move only left motor backward 
   Move.leftMotorBackward();
   Move.leftMotorPWM(DutyCycleB);
   delay(1500);
   Move.leftMotorStop();

   // Delay to slowdown
   delay(100);

   // Move Robot Forward
   Move.robotForward();
   Move.robotPWM(DutyCycleA,DutyCycleB);
   delay(1500);
   Move.robotStop();

   // Delay to slowdown
   delay(100);

   // Move Robot Backward
   Move.robotBackward();
   Move.robotPWM(DutyCycleA,DutyCycleB);
   delay(1500);
   Move.robotStop();

   // Delay to slowdown
   delay(100);

   // Move Robot Right
   Move.robotRight();
   Move.robotPWM(DutyCycleA,DutyCycleB);
   delay(1500);
   Move.robotStop();

   // Delay to slowdown
   delay(100);

   // Move Robot Left 
   Move.robotLeft();
   Move.robotPWM(DutyCycleA,DutyCycleB);
   delay(1500);
   Move.robotStop();

   // Delay to slowdown
   delay(100);
}