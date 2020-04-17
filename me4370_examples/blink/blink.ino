/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  // pinMode(13, OUTPUT);
  DDRB = B11111111;  // sets Arduino pins 1 to 8 as outputs 
  
}

// the loop function runs over and over again forever
void loop() {

  PORTB=B11111111;
  delay(1500);              // wait for a second
  PORTB=B00000000;
  delay(1500);              // wait for a second

}
