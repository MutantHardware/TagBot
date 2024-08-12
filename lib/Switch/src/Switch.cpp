#include "Switch.h"

Switch::Switch(std::initializer_list<int> SWPins, unsigned long debounce) {
  _debounce = debounce;
  _size = SWPins.size();
  _SWPins = new int[_size];
  int index = 0;
  for (int pin : SWPins) {
    pinMode(pin, INPUT);
    _SWPins[index++] = pin;
  }
}

void Switch::position(bool states[]) {
  actualTime = millis();
  for (int i = 0; i < _size; i++) {
    reading[i] = digitalRead(_SWPins[i]);

    if (reading[i] != lastState[i]) {
      lastDebounceTime[i] = actualTime;
    }

    if ((actualTime - lastDebounceTime[i]) > _debounce) {
      states[i] = reading[i];
      if (states[i]) {
        _SW[i] = true;
      }
    }

    lastState[i] = reading[i];
  }
  
  if ((actualTime - lastDebounceTime[_size]) > _debounce) {
    if (!reading[0] && !reading[1]) {
      states[_size] = true;
      _SW[_size] = true; 
    }
    else {
      states[_size] = false;
    }
    lastDebounceTime[_size] = actualTime;
  }
}

bool Switch::changed(){
  if (_SW[0] && _SW[1] || _SW[0] && _SW[2] || _SW[1] && _SW[2]){ 
    return true;
  }
  else{
    return false;
  }
}
