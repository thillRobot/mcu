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

char packet_in[100]; // there is room here for CR NL and NULL TERMINATOR
char * ptr;

char preamble[] = "PREAMBLE\r\n"; // serial packet preamble;
char header[] = "!#HEADER#"; // serial packet header;
char packet_out[100]; // serial packet preamble;

int a,b,c,d,e,f; // globals vars to go to data packet
int g,h,i,j,k,l; // globals vars to come from data packet


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

  a=1;
  b=2;
  c=3;
  d=4;
  e=5;
  f=6;

}

//int cnt;
int reading;

int preamble_returned=0;
int preamble_sent=0;

void loop() { 
   
  if(preamble_returned==0){
    // send the preamble to handshake before you send a data packet
    Serial.print("Sending Preamble\r\n");
    Serial2.print(preamble);
  
  }else{
    // after the handshake, send the data packet
    Serial.print("Sending Packet\r\n");

    build_packet();
    
    Serial.print(packet_out);
    Serial.print("\r\n");
    Serial2.print(packet_out);
 
  }

  // read from buffer until carriage return and newline
  ptr = get_packet(); // returns pointer to string and defines global var 'packet_in'

  // check to see if the preamble was returned
  if ((strcmp(packet_in, preamble)==0)&&(preamble_returned==0)) {
    preamble_returned=1;
    Serial.print("Preamble Returned\r\n");
   
  }else if((preamble_returned==1)){
    Serial.print("Packet Recieved\r\n");

    Serial.print(packet_in);
    Serial.print("\r\n");
    // run the 'checksum'
    int check1,check2;
    
    //check1=calc_checksum(packet_in);
    //Serial.print(check1);
    //Serial.print("\r\n");
    
    //check2=find_checksum(packet_in);    
    //Serial.print(check2);
    //Serial.print("\r\n");
    float a,b,c;
    g=get_data('G'); // get data  from the packet by 'key' name
    h=get_data('H');
    i=get_data('I');
    j=get_data('J'); // get data  from the packet by 'key' name
    k=get_data('K');
    l=get_data('L');
    /*
    Serial.print(d);
    Serial.print("\r\n");
    Serial.print(e);
    Serial.print("\r\n");
    Serial.print(f);
    Serial.print("\r\n");
    */
  }

  Serial.flush();
  Serial2.flush();

  // apparantly this delay is not needed, maybe the debug prints are delay enough
  delay(10);

}

// function to get a single string from the serial line
// Read the string by reading one byte at a time while looking for the carriage return
// populate the string with the individual characters as array
char * get_packet() {

  int k = 0;
  int reading = 1;
  
  int curr_byte = 0;
  int prev_byte = 0;

  while (reading) {

    while (Serial2.available() == 0) ;

    prev_byte = curr_byte;
    curr_byte = Serial2.read();
    packet_in[k] = curr_byte;  // update global var, i would like fix this

    if ((curr_byte == 10) && (prev_byte == 13)) {
      reading = 0;
    }
    k++;
    
  }
  packet_in[k]='\0'; // dont forget the null terminator

}

/*
// function to find the check sum that was sent for packet verification // NEED TO FINISH !
int find_checksum(char *p){

  Serial.println("Finding Checksum");
  
  char curr_byte;
  char prev_byte;
  char next_byte;
 
  char cstr_in[3];

  bool reading=1;
  bool found=0;

  int i=0;
  int j=0;
  
  while(reading){
    
    curr_byte=*(p+i);
    next_byte=*(p+i+1);
    Serial.print("Debug0\r\n");
    Serial.print(prev_byte);
    Serial.print(curr_byte);
    Serial.print(next_byte);
    Serial.print("\r\n");  
        
    if((prev_byte==36)&&(curr_byte==35)){
      
      found=1;
      cstr_in[j]=next_byte;
      cstr_in[j+1]='\0';
      j++;
      
      Serial.print("Debug1\r\n");
      Serial.print(prev_byte);
      Serial.print(curr_byte);
      Serial.print(next_byte);
      Serial.print("\r\n");
      Serial.print(cstr_in);
      Serial.print("\r\n");
    }

    
    if((curr_byte==33)&&(prev_byte==35)){
      
      reading=0;
      Serial.println("Debug2");
       Serial.print(prev_byte);
      Serial.print(curr_byte);
      Serial.print(next_byte);
      Serial.print("\r\n");
    }

    prev_byte=curr_byte;
    i++;
  }
  
  
  return atof(cstr_in);
}
*/
// function to compute and return the check sum for packet verification
int calc_checksum(char *p){

  char curr_byte;
  char prev_byte;
  char next_byte;  

  bool reading=1;
  bool summing=0;
 
  int csum=0;
  int csum_in;
  
  int i=0;
  int j=0;
  
  while(reading){
    
    curr_byte=*(p+i);
    next_byte=*(p+i+1);
    
    if((curr_byte==36)&&(prev_byte==35)){
      summing=1;
    }

    if (summing){     
      csum=csum+curr_byte; //compute the checksum (entire packet up until $#)
    }
    
    if((curr_byte==35)&&(prev_byte==36)){
      reading=0;
      csum=csum-35;
    }
   
    prev_byte=curr_byte;
    i++;
  }
  
  return csum;
}

// function to get a build a data packet
// to begin, this will just populate the global var 'packet_out'
void build_packet(){

  char header[]="!#HEADER#";
  char data[100];
  char check[10];
  char str[100];

  strcpy(str,header);
  sprintf(data,"$A:%d$B:%d$C:%d$D:%d$E:%d$F:%d$#",a,b,c,d,e,f);     
  strcat(str,data);
  sprintf(check,"%d#!\r\n",calc_checksum(str));
  strcat(str,check);
  strcpy(packet_out,str);
  
}

// function to find and return a peice of data in the packet by name
float get_data(char field){

  char dstr[10];
  char curr_byte;
  char prev_byte;

  bool reading=1;
  bool found=0;
  
  int i=0;
  int j=0;
  
  while(reading){

    curr_byte=packet_in[i];

    // wait for the key proceeded by the dollar sign
    if ((curr_byte==field)&&(prev_byte=='$'))
    {
      found=1;      
    } 
    
    // and this happened... TWH (06/27) 
    // 'slice' everything but the first 2 elements and leave off the last

    if ((found==1)&&(reading==1)){
      if ((curr_byte==10)||(curr_byte==36)){ // stop if you find another '$' or a newline
        reading=0;
      }else if (j>0){
        dstr[j-2]=curr_byte;
        dstr[j-1]='\0';
      }
      j++; 
    }
    
    prev_byte=curr_byte;
  
    i++;
  }
  return atof(dstr); // convert string to float and return it
}
