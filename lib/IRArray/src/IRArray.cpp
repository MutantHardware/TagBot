#include "IRArray.h"

// Constructor
IRArray::IRArray(std::initializer_list<int> IRPins, bool isInverted) {
  _size = IRPins.size();
  _IRPins = new int[_size];
  _isInverted = isInverted;
  int index = 0;
  for (int pin : IRPins) {
    pinMode(pin, INPUT);
    _IRPins[index++] = pin;
  }
}

// Method to detect IR Sensor
void IRArray::detect(bool sensorValues[]) {
  for (int i = 0; i < _size; i++) {
    sensorValues[i] = _isInverted ? !digitalRead(_IRPins[i]) : digitalRead(_IRPins[i]);
  }
}
