/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/Blink
*/
/*
low high , low high
none none, low high
high low, low high
high low, none none 
high low, high low
none none, high low
low high, high low
low high, none none  
*/
int halfstep[8]={0b00000101,0b00000001,0b00001001,0b000001000,0b00001010,0b00000010,0b00000110,0b00000100}; 
int step_cnt=0;
int step_idx=0;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
 
  DDRA=0b00001111; // PA0, PA1, PA2 and PA3 as outputs - these are the step signals
  DDRB=0b00000011; // PB0 and PB1 as outputs - these are the enables

  PORTB|=0b00000011; //set the enables high
  
}

// the loop function runs over and over again forever
void loop() {
  
  step_idx=step_cnt%8;
  PORTA=halfstep[step_idx];
  
  step_cnt++;
  delay(10);                       // wait for a second
}
