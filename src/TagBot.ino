/*
MIT License

Copyright (c) 2024 João V. M. Grando

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

GitHub Repository: https://github.com/MutantHardware/TagBot
*/

// Including Libraries
#include "Move.h"
#include "Pinout.h"
#include "Switch.h"
#include <NewPing.h>
#include <Arduino.h>
#include "Sequence.h"
#include <Gyroscope.h>
#include <ESP32Servo.h>
#include "Electromagnet.h"
#include "BluetoothSerial.h"

// Time variables and constants
namespace Time {
  // Electromagnet Time 
  long emg = 0; // Electromagnet Time 
  unsigned long emgLast = 0; // Last Electromagnet Time
  const int dEmg = 100; // Default Electromagnet Time
  
  // Movements Time
  long mov = 0; // Movement Time
  unsigned long movLast = 0; // Last Movement Time

  const int dFwdApp = 2500; // Default Forward App Time
  const int dBwdApp = 300;  // Default Backward App Time
  const int dClkApp = 100; // Default Clockwise App Time
  const int dCcwApp = 100; // Default Counterclockwise App Time
  const int dStpApp = 100;  // Default Stop App Time
    
  const int dFwd = 2500;  // Default Forward Time 
  const int dBwd = 300;  // Default Backward Time
  const int dClk = 1000; // Default Clockwise Time
  const int dCcw = 1000; // Default Counterclockwise Time
  const int dStp = 100;  // Default Stop Time
  
  // Control Time
  const int ctl = 10; // Time delay between control actions

  // Debounce Time
  const int dbc = 50; 
  
  // Servo Time
  const int srv = 500; 

  // Ultrasonic Sensor Time
  const int us = 100;
}

// PWM variables and constants
namespace PWM {
  // Movements PWM
  const int dFwdR = 255; // Default Forward Right Motor PWM
  const int dFwdL = 255; // Default Forward Left Motor PWM
  const int dBwdR = 255; // Default Backward Right Motor PWM
  const int dBwdL = 255; // Default Backward Left Motor PWM
  const int dClwR = 255; // Default Clockwise Right Motor PWM
  const int dClwL = 255; // Default Clockwise Left Motor PWM
  const int dCcwR = 255; // Default Counterclockwise Right Motor PWM
  const int dCcwL = 255; // Default Counterclockwise Left Motor PWM

  // Control PWM (Joystick and Gyroscope)
  const int ctlMin = 200; // Control Minimum Motor PWM
  const int ctlMax = 255; // Control Maximum Motor PWM
  const int ctlThd = 40;  // Control Threshold Motor PWM
  const int ctlDfR = 240; // Control Default Right Motor PWM
  const int ctlDfL = 240; // Control Default Left Motor PWM
}

// Ultrasonic Sensor Variables, range 2~400 cm
namespace Ultrasonic {
  constexpr int minDistF = 8;  // Minimum frontal distance, in cm
  int minDistS = 2 * minDistF; // Minimum side distance, in cm
  constexpr int maxDist = 50;  // Maximum sensor distance, in cm
  bool US[3] = {false, false, false}; // Ultrasonic sensor states
}

// Angle variables and constants
namespace Angle {
  // Servo 
  int angle; // Variable to store servo angle
  const int mid = 90; // Middle Angle Servo
  const int rot = 45; // Angle servo will rotate from middle angle
  
  // Gyroscope
  bool isAngle = false; // Variable to check if is angle or not
  bool rotated = false; // Variable to check if robot finished rotation
  const int error = 2;  // Angle error
}

// Bluetooth variables and constants
namespace Bluetooth {
  int var[4]; 
  boolean newData = false; 
  const byte numChars = 32; 
  char rcvdChars[numChars]; 
  char tempChars[numChars]; 
  bool received;
  bool connected = false;
}

// Sequence variables and constants
namespace Seq {
  // Sequences Length
  int movLen = 0; // Movement Sequence Length
  int emgLen = 0; // Electromagnet Sequence Length
  int seqLen = 0; // Sequence Length
  const int maxLen = 32; // Sequence Maximum Length

  // Sequences states
  bool mov = false; // Movements Sequence
  bool emg = false; // Electromagnet Sequence
  bool clr = false; // Clear Sequence
  bool exe = false; // Execute Sequence
  bool movActive = false; // Movements Execution
  bool emgActive = false; // Electromagnet Execution
}

// Switch variables
bool SW[3] = {false,false,false};

// Creating Servo Object
Servo Servo; 

// Creating Gyro Object
Gyroscope Gyro;

// Creating I2C Object
TwoWire I2C = TwoWire(0);

// Creating Bluetooth Serial Object
BluetoothSerial SerialBT;

// Creating Movements object
Sequence MovSeq(Seq::maxLen);

// Creating Electromagnet object
Sequence EmgSeq(Seq::maxLen);

// Creating Switch Object
Switch Switch({SW0,SW1},Time::dbc);

// Creating Electromagnet Object
Electromagnet Electromagnet(EMG);

// Creating UltrasonicSensor object
NewPing UltrasonicSensor(TRIG,ECHO,Ultrasonic::maxDist);

// Creating Move object
Move Move(AIN1,AIN2,PWMA,BIN1,BIN2,PWMB);

// Default interruption value to wait function 
void wait(unsigned long time, bool interrupt = false);

void setup() {
  // Stop robot
  Move.robotStop();

  // Initializing Serial communication
  Serial.begin(115200);

  // Initializing I2C communication
  I2C.begin(SDA, SCL, 400000);

  // Deactivate Electromagnet
  Electromagnet.deactivate();

  // Register Callback
  SerialBT.register_callback(callback);
  
  // Stay in the loop if gyroscope or Bluetooth doesn't work
  Serial.println("Initialized!");
  while(Gyro.begin(I2C,0x68) != 0 || !SerialBT.begin("TagBot")){

  }  

  // Calculate Offsets and filter
  Gyro.calcOffsets(true,true); 
  Gyro.setFilterGyroCoef(0.95);

  // Allocate Timer
  ESP32PWM::allocateTimer(3);

  // Initialize Servo
  Servo.setPeriodHertz(50);   
  Servo.attach(SRV, 500, 2400); 
  
  // Move servo to the middle position
  Servo.write(Angle::mid);

  wait(Time::srv);

  Serial.println("Finished!");
}

void loop() {
  // Read Switch Position
  Switch.position(SW);
  
  // Bluetooth Modes
  if (Bluetooth::connected) {
    // Call tagBot Function if connected to bluetooth
    tagBot(); 

    // Execute Sequence
    if (Seq::exe) {
      executEmgSeq();
    } 
  }
  // Switch Modes
  else {
    // Choose Mode
    if (SW[0]) {
      // Obstacle avoidance without servo 
      obstacleAvoidance();              
    } 
    else if (SW[1]) {
      // Obstacle avoidance with servo 
      obstacleAvoidanceServo();
    } 
  }

  // Search for switch Interrupt
  switchInterrupt();

  // Clear Sequences
  if (Seq::clr) {
    MovSeq.deleteAll();
    EmgSeq.deleteAll();
    Seq::clr = !Seq::clr;
  }
}

// Function to control obstacle avoidance with servo
void obstacleAvoidanceServo() {
  // Verify middle position crash
  Ultrasonic::US[0] = crashInterrupt(Ultrasonic::minDistF);
  
  // If Robot will crash
  if (Ultrasonic::US[0]) {
    // Stop Robot
    Move.robotStop();
    Serial.println("R: ●");

    // Move Servo to the right position
    Angle::angle = Angle::mid + Angle::rot;
    Servo.write(Angle::angle);
    Serial.println("S: →");
    wait(Time::srv,false);

    // Verify right position crash
    Ultrasonic::US[1] = crashInterrupt(Ultrasonic::minDistS);
    wait(Time::us,false);

    // Move Servo to the left position 
    Angle::angle = Angle::mid - Angle::rot;
    Servo.write(Angle::angle);
    Serial.println("S: ←");
    wait(Time::srv,false);

    // Verify left position crash
    Ultrasonic::US[2] = crashInterrupt(Ultrasonic::minDistS);
    wait(Time::us,false);

    // Move Servo to the middle position
    Servo.write(Angle::mid);
    Serial.println("S: ↑"); 
    wait(Time::srv,false);
 
    // Detected obstacle in the right and in the left
    if (Ultrasonic::US[1] && Ultrasonic::US[2]) {
      // Move robot backward
      Serial.println("R: ↓");
      Move.robotBackward();
      Move.robotPWM(PWM::dFwdR,PWM::dFwdL);
      
      wait(Time::dBwd,false); 

      // Rotate robot clockwise (could be counterclockwise as well)
      Serial.println("R: ↻");
      Move.robotRight();
      Move.robotPWM(PWM::dClwR,PWM::dClwL);

      wait(Time::dClk,false);
    }
    // Detected obstacle in the right 
    else if (Ultrasonic::US[1] && !Ultrasonic::US[2]) {
      // Rotate robot counterclockwise
      Serial.println("R: ↺"); 
      Move.robotLeft();
      Move.robotPWM(PWM::dCcwR,PWM::dCcwL);

      wait(Time::dCcw,false);
    }
    // Detected obstacle in the left
    else if (!Ultrasonic::US[1] && Ultrasonic::US[2]) {
      // Rotate robot clockwise
      Serial.println("R: ↻");
      Move.robotRight();
      Move.robotPWM(PWM::dClwR,PWM::dClwL);

      wait(Time::dClk,false);
    }
    else {
      // Rotate robot clockwise (could be counterclockwise as well)
      Serial.println("R: ↻");
      Move.robotRight();
      Move.robotPWM(PWM::dClwR,PWM::dClwL);

      wait(Time::dCcw,false);
    }
  }
  else {
    // Move Robot Forward
    Move.robotForward();
    Move.robotPWM(PWM::dFwdR,PWM::dFwdL);
    Serial.println("R: ↑");
  }
}

// Function to control obstacle avoidance without servo
void obstacleAvoidance() {
  // Verify crash Interruption
  while (crashInterrupt(Ultrasonic::minDistF)){
    // Stop Robot
    Move.robotStop();
    Serial.println("●");

    wait(Time::dStp,false);

    // Move robot backward
    Move.robotBackward();
    Move.robotPWM(PWM::dBwdR,PWM::dBwdL);
    Serial.println("↓");

    wait(Time::dBwd,false);
    
    // Rotate robot clockwise
    Serial.println("↻");
    Move.robotRight();
    Move.robotPWM(PWM::dClwR,PWM::dClwL);
    
    wait(Time::dClk,true);
  }
  // Move Robot Forward
  Move.robotForward();
  Serial.println("↑");
  Move.robotPWM(PWM::dFwdR,PWM::dFwdL);
}          

// Function to control the robot through TagBot app
void tagBot() {
  // Receive and parse bluetooth serial data
  receive('<','>'); 

  // Execute actions, based in the received values
  if (Bluetooth::received) {
    // Choose the sequence
    Seq::mov = false;
    Seq::emg = false;
    switch(Bluetooth::var[0]) {
      case 1:
        Seq::mov = !Seq::mov;
        break;
      case 2:
        Seq::emg = !Seq::emg;
        break;
      case 3:
        Seq::mov = !Seq::mov;
        Seq::emg = !Seq::emg;
        break;
      default:
        break;
    }

    Serial.println(Seq::mov ? " Movements Sequence" : "");
    Serial.println(Seq::emg ? " Electromagnet Sequence" : "");

    // Verify if is angle or not
    Bluetooth::var[2] == 6 ? Angle::isAngle = true : Angle::isAngle = false;
    
    // Choose the action
    switch (Bluetooth::var[1]) {
      case 1:
        // Execute Function to control robot PWM
        controlPWM(Bluetooth::var[2],Bluetooth::var[3],PWM::ctlThd,PWM::ctlMin,PWM::ctlMax,Time::ctl);
        break;
      case 2:
        // Execute Action Function 
        controlAction(Bluetooth::var[2],Bluetooth::var[3]);
        break;
      case 3:
        // Flag to execute Sequence
        Seq::exe = true;
        break;
      case 4: 
        // Record Sequence Function 
        Serial.println("Recording...");
        Seq::mov ? MovSeq.record(Bluetooth::var[2],Bluetooth::var[3]) : void();
        Seq::emg ? EmgSeq.record(Bluetooth::var[2],Bluetooth::var[3]) : void();
        break;
      case 5:
        // Delete Last Element Function
        Serial.println("Deleting Last Element...");
        Seq::mov ? MovSeq.deleteLast() : void();
        Seq::emg ? EmgSeq.deleteLast() : void();
        break;
      case 6:
        // Clear Sequence function
        Serial.println("Cleaning...");
        Seq::mov ? MovSeq.deleteAll() : void();
        Seq::emg ? EmgSeq.deleteAll() : void();
        Seq::clr = false;
        break;
      default:
        break;
    }
    Bluetooth::received = false;
  }
}

// Function to control the PWM of the wheels
void controlPWM(int pwmA, int pwmB, int threshold, int minPWM, int maxPWM, int time) {
  
  int DutyCycleA = PWM::ctlDfR;
  int DutyCycleB = PWM::ctlDfL;
  
  // Wait a bit
  unsigned long lastTime = 0;
  unsigned long actualTime = millis();
  if (actualTime - lastTime >= time) {
    // Read Switch position
    Switch.position(SW);
    
    // Stop Condition
    if (pwmA == 0 && pwmB == 0 || SW[0] && crashInterrupt(Ultrasonic::minDistF) && pwmB > 0) {
      Serial.print("●");
      Move.robotStop();
      DutyCycleA = pwmA;
      DutyCycleB = pwmA;
    }
    else {
      if (pwmB > 0) {
        Move.robotForward();
        Serial.print("↑ with "); 
        DutyCycleA = pwmB;
        DutyCycleB = pwmB;
      } 
      else if (pwmB < 0) {
        Serial.print("↓ with "); 
        Move.robotBackward();
        DutyCycleA = -pwmB;
        DutyCycleB = -pwmB;
      }

      if (pwmA > 0) {
        DutyCycleA -= pwmA;
        DutyCycleB += pwmA;
      } 
      else if (pwmA < 0) {
        DutyCycleA += abs(pwmA);
        DutyCycleB -= abs(pwmA);
      }
      
      // Choose PWM range
      if (abs(pwmB) < threshold) {
        DutyCycleA = 0;
        DutyCycleB = 0;
      }
      else {
        // Confine the range from min to maxPWM
        DutyCycleA = constrain(DutyCycleA, minPWM, maxPWM);
        DutyCycleB = constrain(DutyCycleB, minPWM, maxPWM);
      }

      // Set PWM
      Move.robotPWM(DutyCycleA, DutyCycleB);
    }
  }
  else {
    Serial.print("●");
    Move.robotStop();
  }
  Serial.println("x: " + String(DutyCycleA) + " y: " + String(DutyCycleB));
}

// Function to control robot actions
void controlAction(int action, int value) {
  // Execute Movement action
  if (Seq::mov) {
    mControlAction(action,value);
    if (!Angle::isAngle) {
      // Read Switch position
      Switch.position(SW);

      // Wait a bit 
      wait(Time::mov,SW[0]);

      // Stop robot
      Move.robotStop();
      Serial.println("●");
    }
  }

  // Execute Electromagnet action
  if (Seq::emg) {
    eControlAction(action,value);

    // Wait a bit 
    wait(Time::emg,SW[0]);

    // Deactivate electromagnet
    Electromagnet.deactivate();
    Serial.println("OFF");
  }
}

// Function to control movements actions
void mControlAction(int mAction, int mValue) {
  // Choose movement
  switch(mAction) {
    case 1:   
      // Make sure time != 0
      mValue == 0 ? Time::mov = Time::dFwdApp : Time::mov = mValue;

      // Move robot forward
      Move.robotForward();
      Serial.println("↑ " + String(Time::mov));

      // Set robot pwm
      Move.robotPWM(PWM::dFwdR,PWM::dFwdL);
      break;
    case 2:    
      // Make sure time != 0
      mValue == 0 ? Time::mov = Time::dBwdApp : Time::mov = mValue;

      // Move robot backward
      Move.robotBackward();
      Serial.println("↓ " + String(Time::mov)); 

      // Set robot pwm
      Move.robotPWM(PWM::dBwdR,PWM::dBwdL);
      break;
    case 3:
      // Make sure time != 0
      mValue == 0 ? Time::mov = Time::dClkApp : Time::mov = mValue;

      // Rotate robot clockwise
      Move.robotRight();
      Serial.println("↻ " + String(Time::mov)); 

      // Set robot pwm
      Move.robotPWM(PWM::dClwR,PWM::dClwL);
      break;
    case 4: 
       // Make sure time != 0
      mValue == 0 ? Time::mov = Time::dCcwApp : Time::mov = mValue;

      // Rotate robot counterclockwise
      Move.robotLeft();
      Serial.println("↺ " + String(Time::mov)); 

      // Set robot pwm
      Move.robotPWM(PWM::dCcwR,PWM::dCcwL);
      break;  
    case 5:
      // Make sure time != 0
      mValue == 0 ? Time::mov = Time::dStpApp : Time::mov = mValue;

      // Stop robot
      Move.robotStop();
      Serial.println("● " + String(Time::mov));
      break;
     case 6:
      // Rotate robot with Angle
      Serial.println("Rotate " + String(mValue) + "°");
      rotate(mValue, Angle::error);
      break;
    default:
      break;
  }
}

// Function to control electromagnet action
void eControlAction(int eAction, int electromagnetTime) {
  // Make sure electromagnet time != 0
  electromagnetTime == 0 ? Time::emg = Time::dEmg : Time::emg = electromagnetTime;
  
  // Choose electromagnet state
  if (eAction == 7) {
    Electromagnet.activate();
    Serial.println("ON " + String(electromagnetTime));
  }
  else if (eAction == 8) {
    Electromagnet.deactivate();
    Serial.println("OFF " + String(electromagnetTime));
  }
}

// Function to execute the electromagnet sequence
void executEmgSeq() {
  // Calculate sequences length
  Seq::movLen = MovSeq.getLength();
  Seq::emgLen = EmgSeq.getLength();
  Seq::seqLen = max(Seq::movLen, Seq::emgLen);
  
  Serial.println("M Length: " + String(Seq::movLen));
  Serial.println("E Length: " + String(Seq::emgLen));
  Serial.println("S Length: " + String(Seq::seqLen));

  // Execute sequences
  Serial.println("Started!");
  int m = 0, e = 0, s = 0;
  while (s < Seq::seqLen) {
    // Only execute if movement is defined and isn't empty
    if (Seq::mov && Seq::movLen != 0) {
      // Movement Actual Time
      unsigned long mActualTime = millis();
      
      // Execute Action if isn't executing and is in range
      if (!Seq::movActive && m < Seq::movLen) {
        MovSeq.executeStep(mControlAction, m);
        Seq::movActive = !Seq::movActive; 
        Time::movLast = mActualTime;    
      }

      // Increase sequence value if is angle and Angle::rotated
      if (Angle::isAngle && Angle::rotated) {
        m++;
      }

      // If isn't angle and is executing, wait untill defined time and stop robot
      if (!Angle::isAngle && Seq::movActive && (mActualTime - Time::movLast >= Time::mov)) {
        Move.robotStop();
        Serial.println("●");
        Seq::movActive = !Seq::movActive;
        m++;  
      }
    }

    // Only execute if electromagnet is defined and isn't empty
    if (Seq::emg && Seq::emgLen != 0) {
      // Electromagnet Actual Time
      unsigned long eActualTime = millis();

      // Execute Action if isn't executing and is in range
      if (!Seq::emgActive && e < Seq::emgLen) {
        EmgSeq.executeStep(eControlAction, e);
        Seq::emgActive = !Seq::emgActive;
        Time::emgLast = eActualTime;
      }
      
      // If is executing, wait untill defined time and deactivate electromagnet
      if (Seq::emgActive && (eActualTime - Time::emgLast >= Time::emg)) {
        Serial.println("OFF");
        Electromagnet.deactivate();
        Seq::emgActive = !Seq::emgActive;
        e++;
      }
    }

    // Read Switch Position
    Switch.position(SW);
    
    // Switch Interruption
    switchInterrupt(); 

    // Break statements, including interrupts
    if (max(m,e) == Seq::seqLen && !Seq::emgActive && !Seq::movActive || bluetoothInterrupt() 
        || Seq::mov && SW[0] && crashInterrupt(Ultrasonic::minDistF) && Bluetooth::var[2] != 2) {
      break;
    }
  }

  Serial.println("Finished Sequence!");

  // Reset States
  Seq::clr = true;
  Seq::exe = false;
  Seq::movActive = false;
  Seq::emgActive = false;

  // Stop Robot
  Move.robotStop();
  
  // Deactivate Electromagnet
  Electromagnet.deactivate();
}

// Function to rotate a desired angle, with error 
void rotate(int desiredAngle, int minError) {
  // Update Gyroscope
  Gyro.update();

  // Get initial angle
  int initialAngle = round(Gyro.AngleZ());

  // Initialize rotation
  Angle::rotated = false;
    
  //while (constrainAngle(initialAngle + desiredAngle) >  constrainAngle(round(Gyro.AngleZ())) - minError) {
  while ((initialAngle + desiredAngle) >  (round(Gyro.AngleZ()) - minError)) {

    // Verify switch interrupt
    switchInterrupt();

    // Interrupt with front sensor if switch is in the middle position or bluetooth interrupt
    if (bluetoothInterrupt() || SW[0] && crashInterrupt(Ultrasonic::minDistF)) {
      break;
    }

    // Rotate robot CCW
    if (desiredAngle > 0) {
      Serial.println("↺");
      Move.robotLeft();
      Move.robotPWM(PWM::dClwR,PWM::dClwL);
    } 
    // Rotate robot CW
    else if (desiredAngle < 0) {
      Serial.println("↻");
      Move.robotRight();
      Move.robotPWM(PWM::dCcwR,PWM::dCcwL); 
    }
    
    // Update Gyroscope
    Gyro.update();
  
    // Get actual angle
    int actualAngle = round(Gyro.AngleZ());

    Serial.println("Actual Angle " + String(actualAngle));
    Serial.println("Desired Angle " + String(desiredAngle));
    Serial.println("Initial Angle " + String(initialAngle));
  }

  // Stop the Robot
  Move.robotStop();
  Serial.println("●");
  
  // Finish rotation
  Angle::rotated = !Angle::rotated; 
}

// Function to verify connection state
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if(event == ESP_SPP_SRV_OPEN_EVT) {
    Bluetooth::connected = !Bluetooth::connected;
  }
  if(event == ESP_SPP_CLOSE_EVT) {
    Bluetooth::connected = !Bluetooth::connected;
    Seq::clr = false;
  }
}

// Function to receive and parse bluetooth serial data
void receive(char startMarker,char endMarker) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    while (SerialBT.available() > 0 && Bluetooth::newData == false) {
      rc = SerialBT.read();
        if (recvInProgress) {
            if (rc != endMarker) {
              Bluetooth::rcvdChars[ndx] = rc;
              ndx++;
              if (ndx >= Bluetooth::numChars){
                ndx = Bluetooth::numChars - 1;
              }
            }
            else {
              Bluetooth::rcvdChars[ndx] = '\0'; 
              recvInProgress = false;
              ndx = 0;
              Bluetooth::newData = true;
            }
        }
        else if (rc == startMarker) {
          recvInProgress = true;
        }
    }
    if (Bluetooth::newData) {
      strcpy(Bluetooth::tempChars, Bluetooth::rcvdChars);
  
      char * strtokrow; 

      strtokrow = strtok(Bluetooth::tempChars, "/"); 

      for (int i = 0; i < 4; i++){
        Bluetooth::var[i] = atoi(strtokrow);
        strtokrow = strtok(NULL, "/"); 
        Serial.print(String(Bluetooth::var[i]) + " ");
      }
      Serial.println("");

      Bluetooth::received = true;           
      Bluetooth::newData = false;          
    }    
}

// Function to verify if bluetooth interrupt was sent
bool bluetoothInterrupt() {
  if (Bluetooth::connected) {
    receive('<','>'); 
    if (Bluetooth::received && Bluetooth::var[1] == 0) {
      return true;
    }
  }
  return false;
}

// Function to verify if switch changed position
void switchInterrupt() {
  // Read Switch Position
  Switch.position(SW);
  
  // Restart ESP if switch changed
  Switch.changed() ? (ESP.restart()) : void();
}

// Function to verify if robot will crash
bool crashInterrupt(int minDist) {
  int distance = UltrasonicSensor.ping_cm();
  return distance > 0 && distance <= minDist ? 1 : 0;
}

// Function to wait a certain time, with interrupt or not
void wait(unsigned long time, bool interrupt) {
  unsigned long actualTime = millis();
  while (actualTime + time > millis()) {
    switchInterrupt(); // Switch Interruption
    if (interrupt && crashInterrupt(Ultrasonic::minDistF) || bluetoothInterrupt()) {
      break;
    }
  }
}
