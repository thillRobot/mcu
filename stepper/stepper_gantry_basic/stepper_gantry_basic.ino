/*
 
  Tennessee Technological University
  Tristan Hill - Jan 23, 2020

  This is example Code for the Fuyu Motion, XYZ gantry table.
  
  This version uses loops and direct register assignments to control the three axes (x,y,z axis) of the machine. 
   
*/



int dt, cnt;
bool dir;

// the setup function runs once when you press reset or power the board
void setup() {

  DDRB=0b11111111; // set Port B to all outputs
  DDRH=0b11111111; // set Port H to all outputs
  
  cnt=0; // counter for loop() 
  dir=0; // direction to step
  dt=5;  // time delay between steps

}


void loop() {

  // reset the loop counter every 1000 steps and change direction
  if (cnt==1000){
    dir=!dir;
    cnt=0;
  }
  
  if (dir){ // if the direction is 1 step this way
    PORTB=0b10100000;
    PORTH=0b01010000;
    delay(dt);
    PORTB=0b11110000;
    PORTH=0b01111000;
    delay(dt);
  }else{ // otherwise if the direction is 0 step the other way
    PORTB=0b00000000;
    PORTH=0b00000000;
    delay(dt);
    PORTB=0b01010000;
    PORTH=0b00101000;
    delay(dt);
  }
  cnt++;
}
