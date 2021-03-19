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
  
void setup() {
  
  //Initialize Serial Monitor and wait for port to open:
  Serial.begin(57600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only
  
  //Initialize serial Port (Tx2,Rx2) and wait for port to open:
  Serial2.begin(57600);
  while (!Serial2);// wait for serial port to connect. Needed for native USB port only

  // prints string without ending carriage return or line break
  Serial2.print("Hello Pi, this is the Mega.\r\n");
  // or print string with ending carriage return and line break
  // Serial2.println("Hello Pi, this is the Mega.");
}

int byte_in; // for incoming serial data
char str_in[100];

int k;
int reading;

void loop() {
  
  delay(10);
  
  //prints string with ending carriage return or line break
  Serial2.println("Hello Pi, this is the Mega.");

  // Read the string by reading one byte at a time while looking for the carriage return
  // populate the string with the individual characters as array
  k=0;
  reading=1;
  while(reading){
    
    while (Serial2.available() == 0) ;
    
    byte_in = Serial2.read();
    str_in[k]=byte_in;
    
    if(byte_in==13){
      reading=0;
    }
    
    k++;    
  }
  
  
  /*
  // Alternatively we could use the 'String' class, this is shown below, but the internet advises to use cstrings instead
  String str_in;
  while (Serial2.available() == 0) ;
  str_in = Serial2.readStringUntil('\r');
  Serial.println(str_in.length());
  */
  
  // show what you got:
  Serial.println("I received: ");
  //Serial.println(byte_in);
  Serial.println(str_in);
  
}
