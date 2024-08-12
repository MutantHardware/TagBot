// #include "Sequence.h"

// // Constructor
// Sequence::Sequence(int maxRow) : _maxRow(maxRow), _row(0) {
//   _action = new int[_maxRow];  // Allocate memory for actions
//   _value = new int[_maxRow];   // Allocate memory for values
// }

// // Destructor
// Sequence::~Sequence() {
//   delete[] _action;  // Deallocate memory for actions
//   delete[] _value;   // Deallocate memory for values
// }

// // Method to record data to sequence
// void Sequence::record(int action, int value) {
//   if (_row < _maxRow) {
//     _action[_row] = action;
//     _value[_row] = value;
//     _row++;
//   }
// }

// // Method to execute the sequence
// void Sequence::execute(FunctionPointer function) {
//   for (int i = 0; i < _row; i++) {
//     function(_action[i], _value[i]);
//     _action[i] = 0;
//     _value[i] = 0;
//   }
//   _row = 0;
// }

// // Method to delete all elements
// void Sequence::deleteAll() {
//   for (int i = 0; i < _row; i++) {
//     _action[i] = 0;
//     _value[i] = 0;
//   }
//   _row = 0;
// }

// // Method to delete last element
// void Sequence::deleteLast() {
//   if (_row > 0) {
//     _row--;
//     _action[_row] = 0;
//     _value[_row] = 0;
//   }
// }

#include "Sequence.h"

// Constructor
Sequence::Sequence(int maxRow) : _maxRow(maxRow), _row(0) {
  _action = new int[_maxRow];  // Allocate memory for actions
  _value = new int[_maxRow];   // Allocate memory for values
}

// Destructor
Sequence::~Sequence() {
  delete[] _action;  // Deallocate memory for actions
  delete[] _value;   // Deallocate memory for values
}

// Method to record data to sequence
void Sequence::record(int action, int value) {
  if (_row < _maxRow) {
    _action[_row] = action;
    _value[_row] = value;
    _row++;
  }
}

// Method to execute the entire sequence
void Sequence::execute(FunctionPointer function) {
  for (int i = 0; i < _row; i++) {
    function(_action[i], _value[i]);
    _action[i] = 0;
    _value[i] = 0;
  }
  _row = 0;
}

// Method to execute one step of the sequence
void Sequence::executeStep(FunctionPointer function, int index) {
  if (index < _row) {
    // Executa a função no índice especificado
    function(_action[index], _value[index]);

    // Limpa o valor no índice especificado
    // _action[index] = 0;
    // _value[index] = 0;
  
    // // Se o índice era o último elemento, decremente _row
    // if (index == _row - 1) {
    //   _row--;
    // }
  }
}

// Method to delete all elements
void Sequence::deleteAll() {
  for (int i = 0; i < _row; i++) {
    _action[i] = 0;
    _value[i] = 0;
  }
  _row = 0;
}

// Method to delete last element
void Sequence::deleteLast() {
  if (_row > 0) {
    _row--;
    _action[_row] = 0;
    _value[_row] = 0;
  }
}

// Method to get the length of the sequence
int Sequence::getLength() {
  return _row;
}

