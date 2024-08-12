#ifndef IRArray_h
#define IRArray_h

#include "Arduino.h"

class IRArray {
  public:
    IRArray(std::initializer_list<int> IRPins,bool isInverted);
    void detect(bool sensorValues[]);
  private:
    int* _IRPins;
    int _size;
    bool _isInverted;
};

#endif
