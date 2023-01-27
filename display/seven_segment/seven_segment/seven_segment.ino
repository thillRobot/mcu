/*
  seven_segment.ino
  This program demonstrates the use of a 5161AS 7 segment display with the Arduino Mega2560
  see xyz.png for wiring diagram
  Tristan Hill - 2023/01/26 
*/

// define a 1D array of integers containing the patterns needed to show the digits 0-9
int digits[10]={0b11000000,0b11111001,0b10100100,0b10110000,0b10011001,0b10010010,0b10000010,0b11111000,0b10000000,0b10011000};

// setup function runs on board power up or reset
void setup() {

  // set data direction register for PORTA to output (Arduino pins 1 to 8)
  DDRA = B11111111;   
 
}

// loop function runs repeatedly forever
void loop() {

  for(int k=0;k<10;k++)
  {
    // set all pins of port A to kth value of digits[]
    PORTA=digits[k];  
    // pause for 1000 milliseconds
    delay(1000);     
  }

}
