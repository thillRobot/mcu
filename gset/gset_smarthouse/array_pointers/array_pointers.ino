/*
  ASCII table

  Prints out byte values in all possible formats:
  - as raw binary values
  - as ASCII-encoded decimal, hex, octal, and binary values

  For more on ASCII, see http://www.asciitable.com and http://en.wikipedia.org/wiki/ASCII

  The circuit: No external hardware needed.

  created 2006
  by Nicholas Zambetti <http://www.zambetti.com>
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ASCIITable

  Modified by Tristan Hill - June 2019
  GSET Smarthouse Project

  This is a demo program for the serial communication Mega <-> Pi using USB

*/
//#include <iostream>
//#include <ctime>

#include <stdio.h>
#include <string.h>


const char * p;

char str[10] = "TEST\r\n"; 

void setup() {

  //Initialize Serial Monitor and wait for port to open:
  Serial.begin(57600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only

  // prints string without ending carriage return or line break
  Serial.print("Hello Serial Monitor, this is me trying to figure out pointers.\r\n");

}

void loop() {

  delay(100);
  // send the preamble to handshake before you send a data packet
  Serial.print("Testing Pointers");
  Serial.print("\r\n");

  p=str;

  Serial.print(p);


}
