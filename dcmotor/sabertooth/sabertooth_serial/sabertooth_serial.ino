/*
  PWM output example - 10 bit
  Tristan Hill - January,15,2016
  interface with 'sabertooth 2x12 v1.0'
  use 'simplified serial' mode

  "Because Sabertooth controls two motors with one 8 byte character, when operating in Simplified
  Serial mode, each motor has 7 bits of resolution. Sending a character between 1 and 127 will
  control motor 1. 1 is full reverse, 64 is stop and 127 is full forward. Sending a character between
  128 and 255 will control motor 2. 128 is full reverse, 192 is stop and 255 is full forward.
  Character 0 (hex 0x00) is a special case. Sending this character will shut down both motors." - datasheet
*/

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX (not used for sabertooth), TX

// user def funs prototypes here
double map_double(double x, double in_min, double in_max, double out_min, double out_max);

double mot1, mot2;
uint8_t mot1_4bit, mot2_4bit;

void setup() {
  // put your setup code here, to run once:

  mySerial.begin(9600);

  // setup serial monitior
  Serial.begin(9600);  

}

void loop() {

  mot1=0; // motor command speed [-1 , +1]
  mot2=0;

  mot1_4bit=map_double(mot1,-1,1,0,127);
  mot2_4bit=map_double(mot2,-1,1,0,127);

  mySerial.write(mot1_4bit);
  delay(100);
  mySerial.write(mot2_4bit|0b10000000);
  delay(100);
  
  //Serial.println(mot1_4bit);
  //Serial.println(mot2_4bit|0b10000000);
   
}

// this fuction maps a double from one range to another
double map_double(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
