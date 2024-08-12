#ifndef Move_h
#define Move_h
#include <Arduino.h>
#include <driver/ledc.h>

class Move{
    private:
        // Global Variables
        int AIN1;
        int AIN2;
        int PWMA;
        int BIN1;
        int BIN2;
        int PWMB;

        // PWM Settings
        const int PWM_CA = 0;  // PWM Channel A
        const int PWM_CB = 3;  // PWM Channel B
        const int PWM_F = 500; // PWM Frequency
        const int PWM_R = 8;   // PWM Resolution
    public:
        // Move Object
        Move(int Pin_AIN1,int Pin_AIN2,int Pin_PWMA,int Pin_BIN1,int Pin_BIN2,int Pin_PWMB);

        // Right Motor Functions  
        void rightMotorForward();
        void rightMotorBackward();
        void rightMotorPWM(int DutyCycleA);
        void rightMotorStop();

        // Left Motor Functions  
        void leftMotorForward();
        void leftMotorBackward();
        void leftMotorPWM(int DutyCycleB);
        void leftMotorStop();
        
        // Robot Functions
        void robotForward();
        void robotBackward();
        void robotRight();
        void robotLeft();
        void robotStop();
        void robotPWM(int DutyCycleA,int DutyCycleB);
};

#endif