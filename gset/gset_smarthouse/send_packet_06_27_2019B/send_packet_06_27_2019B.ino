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

char str_in[100]; // there is room here for CR NL and NULL TERMINATOR
char * p;

char preamble[] = "PREAMBLE\r\n"; // serial packet preamble;
char packet[] = "HEADER$A:00$B:00$C:00$CHECKSUM\r\n"; // serial packet preamble;

void setup() {
 
  //Initialize Serial Monitor and wait for port to open:
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  //Serial1.flush();
  
  //Initialize serial Port (Tx2,Rx2) and wait for port to open:
  Serial2.begin(9600);
  while (!Serial2);// wait for serial port to connect. Needed for native USB port only
  //Serial2.flush();

  // prints string without ending carriage return or line break
  Serial.print("Hello Serial Monitor, this is the Mega.\r\n");
  //Serial2.print("Hello Pi, this is the Mega.\r\n");

}

int k;
int reading;

int preamble_returned=0;
int preamble_sent=0;

void loop() { 
  //Serial.print("******** TOP OF LOOP *********\r\n");
      
  if(preamble_returned==0){
    // send the preamble to handshake before you send a data packet
    Serial.print("Sending Preamble\r\n");

    Serial2.print(preamble);
  
  }else{
    // after the handshake, send the data packet
    Serial.print("Sending Packet\r\n");
 
    Serial2.print(packet);
    
  }

  // read from buffer until carriage return and newline
  p = get_string();
  Serial.print(p);
  Serial.print(str_in);

  // check to see if the preamble was returned
  if ((strcmp(str_in, preamble)==0)&&(preamble_returned==0)) {
    preamble_returned=1;
    Serial.print("Preamble Returned\r\n");
   
  }else if((preamble_returned==1)){
    Serial.print("Packet Returned\r\n");
    Serial.print(str_in);
    Serial.print("\r\n");
    int x;
    x=get_data('A');
  }

  Serial.flush();
  Serial2.flush();
  
  delay(10);

}

/*
// function to get a build a data packet
void build_packet(char feild, int val){

  int reading=1;
  int found=0;
  int j=0;
  char dstr[3];
    
  while(reading){


  
}
*/

// function to get a single string from the serial line
// Read the string by reading one byte at a time while looking for the carriage return
// populate the string with the individual characters as array
char * get_string() {

  int k = 0;
  int reading = 1;
  
  int curr_byte = 0;
  int prev_byte = 0;
  //char str_in[11];

  while (reading) {

    while (Serial2.available() == 0) ;

    prev_byte = curr_byte;
    curr_byte = Serial2.read();
    str_in[k] = curr_byte;  // update global var, i would like fix this

    if ((curr_byte == 10) && (prev_byte == 13)) {
      reading = 0;
    }
    k++;
    
  }
  str_in[k]='\0'; // dont forget the null terminator

  return str_in;

}

int get_data(char field){

  int reading=1;
  int found=0;
  int j=0;
  char dstr[3];
    
  while(reading){

    if (str_in[k]==field)
    {
      //Serial.print(str_in[k]);
      found=1;
        
    }else if ((str_in[k]==10)||(str_in[k]==36)){
      //Serial.print(str_in[k]);
      reading=0;
    }
    // and this happened... TWH (06/27) // this is to 'slice' everything but the first 2 elements and leave off the last
    if (found){
      if ((j>1)&&(reading)){
        dstr[j-2]=str_in[k]; 
      }
      j++;
    }
    
    k++;
  }
  Serial.print("Data parsed from string\r\n");
  Serial.print(dstr);
  Serial.print("\r\n");
}
