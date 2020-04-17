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
int dt;
int cnt;
bool dir;

// the setup function runs once when you press reset or power the board
void setup() {
  
  DDRB=0b11110000;
  
  DDRH=0b01100000;
  
  dt=1;
  
  cnt=0;
  dir=1;
}

// the loop function runs over and over again forever
void loop() {

    if (cnt==3000){
      dir=!dir;
      cnt=0;
    }

    PORTB = 0b01010000|(dir<<7)|(dir<<5);                     // wait for a second
    PORTH = 0b00100000|(dir<<6); 
    delay(dt);                       // wait for a second
    PORTB = 0b00000000|(dir<<7)|(dir<<5); 
    PORTH = 0b00000000|(dir<<6); 
    delay(dt);
    
    cnt++;
    
}
