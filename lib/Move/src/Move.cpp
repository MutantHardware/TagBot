#include "Arduino.h"
#include "Move.h"

Move::Move(int Pin_AIN1,int Pin_AIN2,int Pin_PWMA,int Pin_BIN1,int Pin_BIN2,int Pin_PWMB){
   // Assign Global Pins
   AIN1 = Pin_AIN1;
   AIN2 = Pin_AIN2;
   PWMA = Pin_PWMA;
   BIN1 = Pin_BIN1;
   BIN2 = Pin_BIN2;
   PWMB = Pin_PWMB;

   // Set Pins as OUTPUT
   pinMode(Pin_AIN1, OUTPUT);
   pinMode(Pin_AIN2, OUTPUT);
   pinMode(Pin_BIN1, OUTPUT);
   pinMode(Pin_BIN2, OUTPUT);
   
   // Configure PWM
   ledcSetup(PWM_CA, PWM_F, PWM_R);
   ledcSetup(PWM_CB, PWM_F, PWM_R);
   ledcAttachPin(PWMA, PWM_CA);
   ledcAttachPin(PWMB, PWM_CB); 
}

/* Methods to control right movements */
void Move::rightMotorForward(){
   // Right Motor Forward
   digitalWrite(AIN1, HIGH);
   digitalWrite(AIN2, LOW);
}

void Move::rightMotorBackward(){
   // Right Motor Backward
   digitalWrite(AIN1, LOW);
   digitalWrite(AIN2, HIGH);
}

void Move::rightMotorPWM(int DutyCycleA){
   // PWM Configuration
   ledcWrite(PWM_CA, DutyCycleA);
}

void Move::rightMotorStop(){
   // Right Motor OFF
   digitalWrite(AIN1, LOW);
   digitalWrite(AIN2, LOW);
   rightMotorPWM(0);
}

/* Methods to control left movements */
void Move::leftMotorForward(){
   // Left Motor Forward
   digitalWrite(BIN1, HIGH);
   digitalWrite(BIN2, LOW);
}

void Move::leftMotorBackward(){
   // Left Motor Backward
   digitalWrite(BIN1, LOW);
   digitalWrite(BIN2, HIGH);
}

void Move::leftMotorPWM(int DutyCycleB){
   // PWM Configuration
   ledcWrite(PWM_CB, DutyCycleB);
}

void Move::leftMotorStop(){
   // Left Motor OFF
   digitalWrite(BIN1, LOW);
   digitalWrite(BIN2, LOW);
   leftMotorPWM(0);
}

/* Methods to control robot movements */
void Move::robotForward(){
   // Right Motor Forward
   rightMotorForward();

   // Left Motor Forward
   leftMotorForward();
}

void Move::robotBackward(){
   // Right Motor Backward
   rightMotorBackward();

   // Left Motor Backward
   leftMotorBackward();
}

void Move::robotRight(){
   // Right Motor Backward
   rightMotorBackward();

   // Left Motor Forward
   leftMotorForward();
}

void Move::robotLeft(){
   // Right Motor Forward
   rightMotorForward();

   // Left Motor Backward
   leftMotorBackward();
}

void Move::robotStop(){
   // Right Motor OFF
   rightMotorStop();

   // Left Motor OFF
   leftMotorStop();
}

void Move::robotPWM(int DutyCycleA,int DutyCycleB){
   // PWM Configuration
   ledcWrite(PWM_CA, DutyCycleA);
   ledcWrite(PWM_CB, DutyCycleB);
}