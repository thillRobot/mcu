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

int digits[10]={0b11000000,0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10011000};

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  // pinMode(13, OUTPUT);
  DDRA = B11111111;  // sets Arduino pins 1 to 8 as outputs 
 
}

// the loop function runs over and over again forever
void loop() {
  for(int k=0;k<10;k++)
  {
    PORTA=digits[k];  
    
    delay(1000);     
              // wait for a second
  }
}
