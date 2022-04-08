/*
  set_clear_bit 
  Conditionally sets or clear the the state of a single pin without branching
  repeatedly in a loop

  Tristan Hill - 04/08/2022

  The algorithm was taken from Bit Twiddling from Stanford (https://graphics.stanford.edu/~seander/bithacks.html)

*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  DDRB = 0b11111111;
  PORTB= 0b00000000;
  Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {

  bool f=1; // conditional flag
  
  PORTB^=(-f^PORTB)&(0b01000000);
  Serial.println(PORTB,BIN);
  
  delay(1000);                       // wait for a second

  f=0;
  
  PORTB^=(-f^PORTB)&(0b01000000);
  Serial.println(PORTB,BIN);
  
  delay(1000); 
}
