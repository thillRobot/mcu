/*
 
  Tennessee Technological University - TTU!
  Tristan Hill - Jan 23, 2020

  This is example Code for the Fuyu Motion, XYZ gantry table.
  
  This version uses loops and direct register assignments to control the third axis (z axis) of the machine. 
   
*/



int dt, cnt;
bool dir;

// the setup function runs once when you press reset or power the board
void setup() {

  DDRB=0b11111111; // set Port B to all outputs
  DDRA=0b11111111;
  cnt=0; // counter for loop() 
  dir=0; // direction to step
  dt=5;  // time delay between steps

}


void loop() {

  // reset the loop counter every 1000 steps and change direction
  /*
  if (cnt==1000){
    dir=!dir;
    cnt=0;
  }
  
  if (dir){ // if the direction is 1 step this way
    
    PORTB=0b10000000;
    delay(dt);

    PORTB=0b11010001;
    delay(dt);
    
  }else{ // otherwise if the direction is 0 step the other way
    
    PORTB=0b00000000;
    delay(dt);
    
    PORTB=0b01010001;
    delay(dt);
    
  }
  */
  PORTA=0b00000001; 
  delay(1);
  PORTA=0b00000010; 
  delay(1);

  
  cnt++;
  
}
