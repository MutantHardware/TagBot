#include "Switch.h"

// Definindo os pinos do switch
#define SW0 36
#define SW1 39

Switch Switch({SW0,SW1}, 50); 

bool SW[3] = {false, false, false}; 

void setup() {
  Serial.begin(9600);

  Serial.println("Setup");

  delay(1000);
}

void loop() {
  Switch.position(SW);

  if (SW[0]){
    Serial.println("Modo 2");
  }
  if (SW[1]){
    Serial.println("Modo 1");
  } 
  if (SW[2]){
    Serial.println("Modo 0");
  }

  if (Switch.changed()){
    //ESP.restart();
    Serial.println("Resetar!");
  }
}
