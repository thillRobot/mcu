/*
 * 
  Tennessee Technological University
  Tristan Hill and Stephen Canfield - April, 08 2022
  
  This is example Code for the Fuyu Motion, XYZ gantry table.
 
  This version uses loops and direct register assignments to control the three axes (x,y,z axis) of the machine.

 Hardware connections:
 x axis (two steppers coupled at port): PB7 - dir, PB6 - step
 y axis; PB5 - dir, PB4 - step
 z axis; PH6 - dir, PH5 - step

 Using stepper drive DM556n - stepper online
 dir chang should be ahead of falling Pulse by 5 microseconds
 Pulse width not less than 2.5 microseconds, triggers on falling edge only
   
*/

int dt, cnt;
bool dir;

// the setup function runs once when you press reset or power the board
void setup() {

  DDRB=0b11110000; // set Port B PB7:4 outputs
  DDRH=0b01100000; // set Port H PH6:5 to all outputs
  PORTB = 0b00000000;
  PORTH = 0b00000000;
 
  cnt=0; // counter for loop()
  dir=0; // direction to step
  dt=1;  // time delay between steps

  Serial.begin(9600);

}

void loop() {
  // reset the loop counter every 1000 steps and change direction
  if (cnt==1000){
    dir=!dir;
    cnt=0;
  }
  // set direction bits
   PORTB^=(-dir^PORTB)&(0b10000000); // set direction for PB7
   PORTB^=(-dir^PORTB)&(0b00100000); // set direction for PB5
   PORTH^=(-dir^PORTH)&(0b01000000); // set direction for PH6

  // toggle the step bits
  PORTB^=(1<<6); // toggle bit PB6
  PORTB^=(1<<4); // toggle bit PB4
  PORTH^=(1<<5); // toggle bit PH5

  delay(dt);    
 // Serial.println(PORTB,BIN);
  cnt++;
}
