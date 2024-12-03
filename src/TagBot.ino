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

// Servo Variables
int angle = 0;

// Ultrasonic Sensor Variables
bool US[3] = {false,false,false};

// Rotation variables
bool isAngle = false;
bool rotated = false;

// Time constants
const int fTime = 2500; // Forward 
const int bTime = 2500; // Backward 
const int cTime = 100;  // Clockwise 
const int wTime = 100;  // Counterclockwise
const int sTime = 100;  // Stop

// Time Variables
long mTime = 0; // Movement Time
long eTime = 0; // Electromagnet Time
unsigned long mLastTime = 0;
unsigned long eLastTime = 0;
const int maxRow = 32;

bool clear = false;
bool execute = false;
bool function = false;
const int minDistance = 8;

// PID variables and constants
int pwm, error;

// PWM constants (A -> Right and B -> Left)
const int minPWM = 200;
const int maxPWM = 255;
const int defaultPWMA = 240;
const int defaultPWMB = 235;

// Forward PWMs
const int fPWMA = 255;
const int fPWMB = 255;

// Backward PWMs
const int bPWMA = 255;
const int bPWMB = 255;

// Clocwise PWMs
const int cPWMA = 255;
const int cPWMB = 255;

// Counterclockwise PWMs
const int wPWMA = 255;
const int wPWMB = 255;

// Bluetooth variables and constants
int var[4];
bool received;
bool clean = false;
bool bluetooth = false;
boolean newData = false;
const byte numChars = 32;
char rcvdChars[numChars];
char tempChars[numChars]; 

// Sequence Variables
int mLength = 0;
int eLength = 0;
int sLength = 0;
bool mSeq = false;
bool eSeq = false;
bool mIsRunning = false;
bool eIsRunning = false;      

// Switch Constants
bool SW[3] = {false,false,false};

// Creating Gyro Object
Gyroscope Gyro;

// Creating Servo Object
Servo Servo; 

// Creating I2C Object
TwoWire I2C = TwoWire(0);

// Creating Bluetooth Serial Object
BluetoothSerial SerialBT;

// Creating Movements object
Sequence mSequence(maxRow);

// Creating Magnet object
Sequence eSequence(maxRow);

// Creating Switch Object
Switch Switch({SW0,SW1},50);

// Creating Electromagnet Object
Electromagnet Electromagnet(EMG);

// Creating UltrasonicSensor object
NewPing UltrasonicSensor(TRIG,ECHO,50);

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

   //Allow allocation of all timers
  // ESP32PWM::allocateTimer(0);
  // ESP32PWM::allocateTimer(1);
  // ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Initialize Servo
  Servo.setPeriodHertz(50);   
  Servo.attach(SRV, 500, 2400); 
  
  // Move servo to the middle position
  Servo.write(90);
  delay(500);

  Serial.println("Finished!");
}

void loop() {
  // Read Switch Position
  Switch.position(SW);
  
  // Bluetooth Modes
  if (bluetooth) {
    // Call tagBot Function if connected to bluetooth
    tagBot(); 

    // Execute Sequence
    if (execute) {
      executeSequence();
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
  if (clear) {
    mSequence.deleteAll();
    eSequence.deleteAll();
    clear = !clear;
  }
}

void obstacleAvoidanceServo() {
  // Verify middle position crash
  US[0] = crashInterrupt(minDistance);
  
  // If Robot will crash
  if (US[0]) {
    // Stop Robot
    Move.robotStop();
    Serial.println("R: ●");

    // Move Servo to the right position
    Servo.write(180);
    Serial.println("S: →");
    wait(500,false);

    // Verify right position crash
    US[1] = crashInterrupt(minDistance);

    // Move Servo to the left position
    Servo.write(0);
    Serial.println("S: ←");
    wait(500,false);

    // Verify left position crash
    US[2] = crashInterrupt(minDistance);

    // Move Servo to the middle position
    Servo.write(90);
    Serial.println("S: ↑"); 
    wait(500,false);
 
    // Update robot PWM
    Move.robotPWM(defaultPWMA,defaultPWMB);

    // Detected obstacle in the right and in the left
    if (US[1] && US[2]) {
      // Move robot backward
      Serial.println("R: ↓");
      Move.robotBackward();
      
      wait(1000,false); 

      // Rotate robot clockwise (could be counterclockwise as well)
      Serial.println("R: ↻");
      Move.robotRight();

      wait(1000,false);
    }
    // Detected obstacle in the right 
    else if (US[1] && !US[2]) {
      // Rotate robot counterclockwise
      Serial.println("R: ↺"); 
      Move.robotLeft();

      wait(1000,false);
    }
    // Detected obstacle in the left
    else if (!US[1] && US[2]) {
      // Rotate robot clockwise
      Serial.println("R: ↻");
      Move.robotRight();

      wait(1000,false);
    }
    else {
      // Rotate robot clockwise (could be counterclockwise as well)
      Serial.println("R: ↻");
      Move.robotRight();

      wait(1000,false);
    }
  }
  else {
    // Move Robot Forward
    Move.robotForward();
    Move.robotPWM(defaultPWMA,defaultPWMB);
    Serial.println("R: ↑");
  }
}

// Ultrasonic Mode Without Servo Function
void obstacleAvoidance() {
  // Verify crash Interruption
  while (crashInterrupt(minDistance)){
    // Stop Robot
    Move.robotStop();
    Serial.println("●");

    wait(100,false);

    // Move robot backward
    Move.robotBackward();
    Move.robotPWM(defaultPWMA,defaultPWMB);
    Serial.println("↓");

    wait(300,false);
    
    // Rotate robot clockwise
    Serial.println("↻");
    Move.robotRight();
    Move.robotPWM(defaultPWMA,defaultPWMB);
    
    //wait(500,true);
    wait(1000,true);
  }
  // Move Robot Forward
  Move.robotForward();
  Serial.println("↑");
  Move.robotPWM(defaultPWMA,defaultPWMB);
}          

// Function to control robot through tagBot app
void tagBot() {
  // Receive and parse bluetooth serial data
  receive('<','>'); 

  // Execute actions, based in the received values
  if (received) {
    // Choose the sequence
    mSeq = false;
    eSeq = false;
    switch(var[0]) {
      case 1:
        mSeq = !mSeq;
        break;
      case 2:
        eSeq = !eSeq;
        break;
      case 3:
        mSeq = !mSeq;
        eSeq = !eSeq;
        break;
      default:
        break;
    }

    Serial.println(mSeq ? " Movements Sequence" : "");
    Serial.println(eSeq ? " Electromagnet Sequence" : "");

    // Verify if is angle or not
    var[2] == 6 ? isAngle = true : isAngle = false;
    
    // Choose the action
    switch (var[1]) {
      case 1:
        // Execute Function to control robot PWM
        controlPWM(var[2],var[3],40,minPWM,maxPWM,10);
        break;
      case 2:
        // Execute Action Function 
        controlAction(var[2],var[3]);
        break;
      case 3:
        // Flag to execute Sequence
        execute = true;
        break;
      case 4: 
        // Record Sequence Function 
        Serial.println("Recording...");
        mSeq ? mSequence.record(var[2],var[3]) : void();
        eSeq ? eSequence.record(var[2],var[3]) : void();
        break;
      case 5:
        // Delete Last Element Function
        Serial.println("Deleting Last Element...");
        mSeq ? mSequence.deleteLast() : void();
        eSeq ? eSequence.deleteLast() : void();
        break;
      case 6:
        // Clear Sequence function
        Serial.println("Cleaning...");
        mSeq ? mSequence.deleteAll() : void();
        eSeq ? eSequence.deleteAll() : void();
        clear = false;
        break;
      default:
        break;
    }
    received = false;
  }
}

// Function to control robot with time
void controlPWM(int pwmA, int pwmB, int threshold, int minPWM, int maxPWM, int time) {
  
  int DutyCycleA = defaultPWMA;
  int DutyCycleB = defaultPWMB;
  
  // Wait a bit
  unsigned long lastTime = 0;
  unsigned long actualTime = millis();
  if (actualTime - lastTime >= time) {
    // Read Switch position
    Switch.position(SW);
    
    // Stop Condition
    if (pwmA == 0 && pwmB == 0 || SW[0] && crashInterrupt(minDistance) && pwmB > 0) {
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

// Function to control robot action
void controlAction(int action, int value) {
  // Execute Movement action
  if (mSeq) {
    mControlAction(action,value);
    if (!isAngle) {
      // Read Switch position
      Switch.position(SW);

      // Wait a bit 
      wait(mTime,SW[0]);

      // Stop robot
      Move.robotStop();
      Serial.println("●");
    }
  }

  // Execute Electromagnet action
  if (eSeq) {
    eControlAction(action,value);

    // Wait a bit 
    wait(eTime,SW[0]);

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
      mValue == 0 ? mTime = fTime : mTime = mValue;

      // Move robot forward
      Move.robotForward();
      Serial.println("↑ " + String(mTime));

      // Set robot pwm
      Move.robotPWM(fPWMA,fPWMB);
      break;
    case 2:    
      // Make sure time != 0
      mValue == 0 ? mTime = bTime : mTime = mValue;

      // Move robot backward
      Move.robotBackward();
      Serial.println("↓ " + String(mTime)); 

      // Set robot pwm
      Move.robotPWM(bPWMA,bPWMB);
      break;
    case 3:
      // Make sure time != 0
      mValue == 0 ? mTime = cTime : mTime = mValue;

      // Rotate robot clockwise
      Move.robotRight();
      Serial.println("↻ " + String(mTime)); 

      // Set robot pwm
      Move.robotPWM(cPWMA,cPWMB);
      break;
    case 4: 
       // Make sure time != 0
      mValue == 0 ? mTime = wTime : mTime = mValue;

      // Rotate robot counterclockwise
      Move.robotLeft();
      Serial.println("↺ " + String(mTime)); 

      // Set robot pwm
      Move.robotPWM(wPWMA,wPWMB);
      break;  
    case 5:
      // Make sure time != 0
      mValue == 0 ? mTime = sTime : mTime = mValue;

      // Stop robot
      Move.robotStop();
      Serial.println("● " + String(mTime));
      break;
     case 6:
      // Rotate robot with Angle
      Serial.println("Rotate " + String(mValue) + "°");
      rotate(mValue, 2, 200, 200);
      break;
    default:
      break;
  }
}

// Function to control electromagnet states
void eControlAction(int eAction, int electromagnetTime) {
  // Make sure electromagnet time != 0
  electromagnetTime == 0 ? eTime = fTime : eTime = electromagnetTime;
  
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

// Function to execute Sequence
void executeSequence() {
  // Calculate sequences length
  mLength = mSequence.getLength();
  eLength = eSequence.getLength();
  sLength = max(mLength, eLength);
  
  Serial.println("M Length: " + String(mLength));
  Serial.println("E Length: " + String(eLength));
  Serial.println("S Length: " + String(sLength));

  // Execute sequences
  Serial.println("Started!");
  int m = 0, e = 0, s = 0;
  while (s < sLength) {
    // Only execute if movement is defined and isn't empty
    if (mSeq && mLength != 0) {
      // Movement Actual Time
      unsigned long mActualTime = millis();
      
      // Execute Action if isn't executing and is in range
      if (!mIsRunning && m < mLength) {
        mSequence.executeStep(mControlAction, m);
        mIsRunning = !mIsRunning; 
        mLastTime = mActualTime;    
      }

      // Increase sequence value if is angle and rotated
      if (isAngle && rotated) {
        m++;
      }

      // If isn't angle and is executing, wait untill defined time and stop robot
      if (!isAngle && mIsRunning && (mActualTime - mLastTime >= mTime)) {
        Move.robotStop();
        Serial.println("●");
        mIsRunning = !mIsRunning;
        m++;  
      }
    }

    // Only execute if electromagnet is defined and isn't empty
    if (eSeq && eLength != 0) {
      // Electromagnet Actual Time
      unsigned long eActualTime = millis();

      // Execute Action if isn't executing and is in range
      if (!eIsRunning && e < eLength) {
        eSequence.executeStep(eControlAction, e);
        eIsRunning = !eIsRunning;
        eLastTime = eActualTime;
      }
      
      // If is executing, wait untill defined time and deactivate electromagnet
      if (eIsRunning && (eActualTime - eLastTime >= eTime)) {
        Serial.println("OFF");
        Electromagnet.deactivate();
        eIsRunning = !eIsRunning;
        e++;
      }
    }

    // Read Switch Position
    Switch.position(SW);
    
    // Switch Interruption
    switchInterrupt(); 

    // Break statements, including interrupts
    if (max(m,e) == sLength && !eIsRunning && !mIsRunning || bluetoothInterrupt() || mSeq && SW[0] && crashInterrupt(minDistance) && var[2] != 2) {
      break;
    }
  }

  Serial.println("Finished Sequence!");

  // Reset States
  clear = true;
  execute = false;
  mIsRunning = false;
  eIsRunning = false;

  // Stop Robot
  Move.robotStop();
  
  // Deactivate Electromagnet
  Electromagnet.deactivate();
}

// Rotate Function
void rotate(int desiredAngle, int minError, int DutyCycleA, int DutyCycleB) {
  // Update Gyroscope
  Gyro.update();

  // Get initial angle
  int initialAngle = round(Gyro.AngleZ());

  // Initialize rotation
  rotated = false;
    
  //while (constrainAngle(initialAngle + desiredAngle) >  constrainAngle(round(Gyro.AngleZ())) - minError) {
  while ((initialAngle + desiredAngle) >  (round(Gyro.AngleZ()) - minError)) {

    // Verify switch interrupt
    switchInterrupt();

    // Interrupt with front sensor if switch is in the middle position or bluetooth interrupt
    if (bluetoothInterrupt() || SW[0] && crashInterrupt(minDistance)) {
      break;
    }

    // Rotate robot CCW
    if (desiredAngle > 0) {
      Move.robotLeft();
      Serial.println("↺");
    } 
    // Rotate robot CW
    else if (desiredAngle < 0) {
      Move.robotRight();
      Serial.println("↻");
    }
    
    // Update Gyroscope
    Gyro.update();
  
    // Get actual angle
    int actualAngle = round(Gyro.AngleZ());

    Serial.println("Actual Angle " + String(actualAngle));
    Serial.println("Desired Angle " + String(desiredAngle));
    Serial.println("Initial Angle " + String(initialAngle));

    // Send PWM to motors
    Move.robotPWM(DutyCycleA, DutyCycleB);  
  }

  // Stop the Robot
  Move.robotStop();
  Serial.println("Parou");
  
  // Finish rotation
  rotated = !rotated; 
}

// Function to constrain angle between 0 and 360
int constrainAngle(int angle){
  if (abs(angle) > 360) {
    angle = angle % 360;
  }
  return abs(angle);
}

// Bluetooth Callback function
void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  if(event == ESP_SPP_SRV_OPEN_EVT) {
    bluetooth = !bluetooth;
  }
  if(event == ESP_SPP_CLOSE_EVT) {
    bluetooth = !bluetooth;
    clear = false;
  }
}

// Function to receive bluetooth data
void receive(char startMarker,char endMarker) {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char rc;

    while (SerialBT.available() > 0 && newData == false) {
      rc = SerialBT.read();
        if (recvInProgress) {
            if (rc != endMarker) {
              rcvdChars[ndx] = rc;
              ndx++;
              if (ndx >= numChars){
                ndx = numChars - 1;
              }
            }
            else {
              rcvdChars[ndx] = '\0'; 
              recvInProgress = false;
              ndx = 0;
              newData = true;
            }
        }
        else if (rc == startMarker) {
          recvInProgress = true;
        }
    }
    if (newData) {
      strcpy(tempChars, rcvdChars);
  
      char * strtokrow; 

      strtokrow = strtok(tempChars, "/"); 

      for (int i = 0; i < 4; i++){
        var[i] = atoi(strtokrow);
        strtokrow = strtok(NULL, "/"); 
        Serial.print(String(var[i]) + " ");
      }
      Serial.println("");

      received = true;           
      newData = false;          
    }    
}

// Bluetooth Interrupt Function
bool bluetoothInterrupt() {
  if (bluetooth) {
    receive('<','>'); 
    if (received && var[1] == 0) {
      return true;
    }
  }
  return false;
}

// Switch Interrupt Function
void switchInterrupt() {
  // Read Switch Position
  Switch.position(SW);
  
  // Restart ESP if switch changed
  Switch.changed() ? (ESP.restart()) : void();
}

// Crash Interrupt Function
bool crashInterrupt(int MinDistance) {
  //int distance = NewPing::convert_cm(UltrasonicSensor.ping_median(10));
  int distance = UltrasonicSensor.ping_cm();
  return distance > 0 && distance <= MinDistance ? 1 : 0;
}

// Fall Interrupt Function
bool fallInterrupt(float MaxPitchAbs, float MaxRollAbs){
  // Update Gyro 
  Gyro.update();

  return abs(Gyro.AngleX()) >= MaxPitchAbs || abs(Gyro.AngleY()) >= MaxRollAbs ? 1 : 0;
}

// Wait Function
void wait(unsigned long time, bool interrupt) {
  unsigned long actualTime = millis();
  while (actualTime + time > millis()) {
    switchInterrupt(); // Switch Interruption
    if (interrupt && crashInterrupt(minDistance) || bluetoothInterrupt()) {
      break;
    }
  }
}