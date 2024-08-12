#include <IRArray.h>

// Defining the IR Pins
#define OUT1 18 // GPIO18
#define OUT2 19 // GPIO19
#define OUT3 21 // GPIO21
#define OUT4 22 // GPIO22
#define OUT5 23 // GPIO23

// Creating the vector to receive the sensor values
bool IR[5];

int sizeOfIR;

// Creating the IRArray Object
IRArray IRArray({OUT1, OUT2, OUT3, OUT4, OUT5},false);

void setup() {
  // Initializing Serial Communication
  Serial.begin(9600);
}

void loop() {
  // Detect the IR Array states
  IRArray.detect(IR);

  // Obtain the size of the IR vector
  sizeOfIR = sizeof(IR) / sizeof(IR[0]);

  // Construct a string containing all sensor values
  String IRValues = "Sensor Values: ";
  for (int i = 0; i < sizeOfIR; i++) {
    IRValues += "Pin ";
    IRValues += i ; 
    IRValues += ": ";
    IRValues += (IR[i] ? 1 : 0);
    if (i < sizeOfIR - 1) {
      IRValues += ", "; 
    }
  }
  
  // Print the sensor values string
  Serial.println(IRValues);

  // Delay for readability
  delay(1000); 
}
