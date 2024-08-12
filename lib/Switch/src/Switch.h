#ifndef Switch_h
#define Switch_h

#include "Arduino.h"

class Switch {
  public:
    Switch(std::initializer_list<int> IRPins,unsigned long debounce);
    void position(bool states[]);
    bool changed();
  private:
    int* _SWPins;
    int _size;
    bool reading[2] = {false,false};
    bool lastState[2] = {false,false};
    bool _SW[3] = {false, false, false};
    unsigned long actualTime = 0;
    unsigned long _debounce;    
    unsigned long lastDebounceTime[3] = {0,0,0};
};

#endif