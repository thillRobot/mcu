// ISR example 
// original code from arduino reference: 
// https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
// Tristan Hill, Septemeber 09,2024
// testing on arduino mkrwifi1010

#include "wiring_private.h"
#include "sam.h"

const byte ledPin = 6;
const byte interruptPin4 = 4;
const byte interruptPin5 = 5;

volatile byte state = LOW;
             
void setup() {
  
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin4, INPUT_PULLUP);
  pinMode(interruptPin4, INPUT_PULLUP);

  //attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
  
  attachInterrupt(interruptPin4, isr4, RISING);
  attachInterrupt(interruptPin5, isr5, RISING);

}

void loop() {
  digitalWrite(ledPin, state);

}

void isr4() {
  state = !state;
}

void isr5() {
  //state = !state;
}

