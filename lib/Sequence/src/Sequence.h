// #ifndef SEQUENCE_H
// #define SEQUENCE_H

// #include "Arduino.h"

// typedef void (*FunctionPointer)(int, int);

// class Sequence {
//   public:
//     // Constructor
//     Sequence(int maxRow);

//     // Destructor
//     ~Sequence();
    
//     // Record an action to the sequence
//     void record(int action, int value);
    
//     // Execute the sequence using the provided function
//     void execute(FunctionPointer function);
    
//     // Delete all the sequence (set all actions and values to 0)
//     void deleteAll();
    
//     // Remove the last action from the sequence
//     void deleteLast();
      
//   private:
//     int* _action;  // Dynamically allocated array to store actions
//     int* _value;   // Dynamically allocated array to store values
//     int _row;
//     int _maxRow;
// };

// #endif // SEQUENCE_H


#ifndef SEQUENCE_H
#define SEQUENCE_H

#include "Arduino.h"

typedef void (*FunctionPointer)(int, int);

class Sequence {
  public:
    // Constructor
    Sequence(int maxRow);

    // Destructor
    ~Sequence();
    
    // Record an action to the sequence
    void record(int action, int value);
    
    // Execute the entire sequence using the provided function
    void execute(FunctionPointer function);

    // Start the sequence execution
    void startExecution(FunctionPointer function);

    // Execute one step of the sequence
    void executeStep(FunctionPointer function, int index);
    
    // Delete all elements in the sequence
    void deleteAll();
    
    // Remove the last action from the sequence
    void deleteLast();

    // Get the length of the sequence
    int getLength();
      
  private:
    int* _action;  // Dynamically allocated array to store actions
    int* _value;   // Dynamically allocated array to store values
    int _row;
    int _maxRow;
};

#endif // SEQUENCE_H
