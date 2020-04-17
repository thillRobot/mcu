/*
 
  Tennessee Technological University
  Tristan Hill - Jan 23, 2020

  This is example Code for the Fuyu Motion, XYZ gantry table.
  
  This version uses bitwise operators to control the third axis (z axis) of the machine. 
   
*/

int dt, cnt;
bool dir;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  DDRB=0b11111111; // set Port B to all outputs
  
  cnt=0; // counter for loop() 
  dir=1; // direction to step
  dt=5;  // time delay between steps

}

void loop() {

  if (cnt==1000){
    dir=!dir;
    cnt=0;
  }

  if (dir){
    PORTB|=(1<<7);
  }else{
    PORTB&=~((1<<7));
  }
  
  PORTB&=~((1<<6));
  delay(dt); 
  PORTB|=(1<<6);
  delay(dt);

  cnt++;
}
