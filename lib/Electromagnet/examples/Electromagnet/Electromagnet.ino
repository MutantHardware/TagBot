// Including Library
#include <Electromagnet.h>

// Electromagnet Pinout
#define ElectromagnetPin 02 // GPIO02

// Creating Electromagnet Object
Electromagnet Electromagnet(ElectromagnetPin);

void setup() {
  Electromagnet.deactivate();
}
 
void loop() {
 Electromagnet.activate();
 delay(2000);
 Electromagnet.deactivate();
 delay(2000);
}