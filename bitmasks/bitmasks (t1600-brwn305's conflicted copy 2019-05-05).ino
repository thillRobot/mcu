/*
  Binary Bit Masks Example

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016
  Updated - February 2, 2018
   
*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  // pinMode(13, OUTPUT);
  //DDRB = DDRB | 00000001;  // sets Port B output
  DDRB = 0b11111111;
  DDRC = 0b00000000;  // sets Port C as input
  
  //PORTC = 0b11111111;

}

// the loop function runs over and over again forever
void loop() {

  while(PINC&0b00000001)
  {
    PORTB|=0b00000001;
  }
  PORTB&=0b111111110;

}
