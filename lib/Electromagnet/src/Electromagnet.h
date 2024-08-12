#ifndef Electromagnet_h
#define Electromagnet_h
#include "Arduino.h"

class Electromagnet{
  private:
    int pin_Electromagnet;
  public:
    Electromagnet(int ElectromagnetPin);
    void activate();
    void deactivate();
};

#endif