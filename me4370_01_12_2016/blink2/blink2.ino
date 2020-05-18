/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Simple Register Level IO example
  Tristan Hill - Jan, 11, 2016
 */


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  // pinMode(13, OUTPUT);
  DDRB = B11111111;  // sets Arduino pins 1 to 8 as outputs 
  DDRE = B11111111;
}

// the loop function runs over and over again forever
void loop() {

  PORTB=B11111111;
  PORTE=B00000000;
  delay(1500);              // wait for a second
  PORTB=B00000000;
  PORTE=B00000001;
  delay(1500);              // wait for a second

}
