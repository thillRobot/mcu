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
#include <stdio.h>
#include <string.h>

int curr_byte = 0; // for incoming serial data
int prev_byte = 0;
char str_in[10];
const char * t;

char preamble[10] = "preamble"; // serial packet preamble;


void setup() {

  //Initialize Serial Monitor and wait for port to open:
  Serial.begin(57600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only

  //Initialize serial Port (Tx2,Rx2) and wait for port to open:
  Serial2.begin(57600);
  while (!Serial2);// wait for serial port to connect. Needed for native USB port only

  // prints string without ending carriage return or line break
  Serial.print("Hello Serial Monitor, this is the Mega.\r\n");
  Serial2.print("Hello Pi, this is the Mega.\r\n");

}



int k;
int reading;

void loop() {

  delay(100);
  // send the preamble to handshake before you send a data packet
  Serial2.print(preamble);
  Serial2.print("\r\n");

  //prints string with ending carriage return or line break

  t=get_string();
  for(int k=0;k<10;k++){
    str_in[k]=*(t+k);
  }
  /*
  k = 0;
  reading = 1;

  while (reading) {

    while (Serial2.available() == 0) ;

    prev_byte = curr_byte;
    curr_byte = Serial2.read();
    str_in[k] = curr_byte;
    //Serial.println(str_in);

    if ((curr_byte == 10) && (prev_byte == 13)) {
      reading = 0;
    }
    k++;
  }
  */

  if (strcmp(str_in, preamble)) {
    Serial.println("Preamble Returned!");
  }

  // show what you got:
  //Serial.println("I received: ");
  //Serial.println(byte_in);
  Serial.println(str_in);



}


// function to get a single string from the serial line
// Read the string by reading one byte at a time while looking for the carriage return
// populate the string with the individual characters as array
const char * get_string(void) {

  int k=0;
  int curr=0;
  int prev=0;
  int reading=1;
  
  while(reading){
  
    while (Serial2.available() == 0) ;
  
    prev=curr;
    curr = Serial2.read();
    str_in[k]=curr;
  
    if ((curr == 10) && (prev == 13)) {
      reading = 0;
      //Serial.println("DEBUG!");
    }
    k++;
  }
  return str_in;

}



// Alternatively we could use the 'String' class, this is shown below, but the internet advises to use cstrings instead
/*
  String str_in;
  while (Serial2.available() == 0) ;
  str_in = Serial2.readStringUntil('\r');
  Serial.println(str_in.length());
*/
