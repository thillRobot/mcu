// ISR example 
// original code from arduino reference: 
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
// Tristan Hill, Septemeber 09,2024
// testing on arduino mkrwifi1010

#include "wiring_private.h"
#include "sam.h"

const byte ledPin = 6;
const byte interruptPin = A2; 
volatile byte state = LOW;
             
void setup() {
  
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  
  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
  attachInterrupt(interruptPin, blink, CHANGE);
 
}

void loop() {
  digitalWrite(ledPin, state);

}

void blink() {
  state = !state;
}

