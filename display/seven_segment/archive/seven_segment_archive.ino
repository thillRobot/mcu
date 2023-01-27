/*
  
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
