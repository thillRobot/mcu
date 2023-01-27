/*
  seven_segment.ino
  This program demonstrates the use of a 5161AS 7 segment display with the Arduino Mega2560
  LEDs are wired with common cathode (-), see 5161AS_wiring.png for wiring diagram
  Tristan Hill - 2023/01/26 
*/

// define a 1D array of integers containing the patterns needed to show the digits 0-9
//digits: {0bABCDEFGP(LEDs)};
int digits[10]={0b11111111,0b00000000,0b11111111,0b00000000,0b11111111,0b00000000,0b11111111,0b00000000,0b11111111,0b00000000};


// setup function runs on board power up or reset
void setup() {

  // set data direction register for PORTA to output (Arduino pins 1 to 8)
  DDRC = 0b11111111;   
 
}

// loop function runs repeatedly forever
void loop() {

  for(int k=0;k<10;k++)
  {
    // set all pins of port A to kth value of digits[]
    PORTC=digits[k];  
    // pause for 1000 milliseconds
    delay(1000);     
  }

}
