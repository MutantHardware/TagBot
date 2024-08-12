#include "Sequence.h"

// Define a sample function to use with the Sequence execute method
void sampleFunction(int action, int value) {
  // Print the action and value to the Serial monitor
  Serial.print("Action: ");
  Serial.print(action);
  Serial.print(", Value: ");
  Serial.println(value);
}

Sequence sequence(10); // Create a Sequence object with a maximum of 10 rows

void setup() {
  // Initialize the Serial communication
  Serial.begin(9600);

  // Record some actions and values into the sequence
  sequence.record(1, 100);
  sequence.record(2, 200);
  sequence.record(3, 300);

  // Execute the sequence using the sampleFunction
  Serial.println("Executing sequence:");
  sequence.execute(sampleFunction);

  // Record more actions and values
  sequence.record(4, 400);
  sequence.record(5, 500);

  // Delete the last action
  sequence.deleteLast();

  // Execute the sequence again
  Serial.println("Executing sequence after deleting last action:");
  sequence.execute(sampleFunction);

  // Record more actions and values
  sequence.record(6, 600);
  sequence.record(7, 700);

  // Delete all actions
  sequence.deleteAll();

  // Try to execute the sequence after deleting all actions
  Serial.println("Executing sequence after deleting all actions:");
  sequence.execute(sampleFunction);
}

void loop() {
  // Nothing to do here
}
