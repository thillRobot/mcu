/*
  Toggle Bit
  Toggles the state of a single pin repeatedly in a loop

  Tristan Hill - 04/08/2022

  The algorithm was taken from geeksforgeeks (https://www.geeksforgeeks.org/set-clear-and-toggle-a-given-bit-of-a-number-in-c/)

*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  DDRB = 0b11111111;
  PORTB= 0b11111111;
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {

  int K=2; // bit to be toggled
  PORTB^=(1<<K);
  Serial.println(PORTB,BIN);
  
  delay(1000);                       // wait for a second
}
