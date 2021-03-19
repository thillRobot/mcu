/*
  Binary Bit Masks Example

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016
   
*/


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  // pinMode(13, OUTPUT);
  DDRB = DDRB | 00000001;  // sets Arduino pins 2 to 3 as outputs 
  //DDRE = B00000000;
}

// the loop function runs over and over again forever
void loop() {

  PORTB=PORTB|B00000001;
 // PORTE=B00000000;
  delay(1500);              // wait for a second
  PORTB=PORTB&111111110;
 // PORTE=B00000001;
  delay(1500);              // wait for a second

}
