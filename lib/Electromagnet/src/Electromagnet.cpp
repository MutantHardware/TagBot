#include "Electromagnet.h"
#include "Arduino.h"

Electromagnet::Electromagnet(int ElectromagnetPin){
  pin_Electromagnet = ElectromagnetPin;
  pinMode(pin_Electromagnet,OUTPUT);
}

void Electromagnet::activate(){
  digitalWrite(pin_Electromagnet,HIGH);
}

void Electromagnet::deactivate(){
  digitalWrite(pin_Electromagnet,LOW);
}