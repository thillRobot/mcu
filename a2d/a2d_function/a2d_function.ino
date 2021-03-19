/*
  Analog to Digital Converter - Function Level Programming
  Multiple Channels with Simple 8bit read
  
  Tennessee Technological University
  Tristan Hill - Jan 12, 2016
   
*/

int val;
int s_pin0 = A0;
int s_pin1 = A1;

// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize digital pin 53(PB0) and pin 52(PB1) as an outputs(1).
  pinMode(52,OUTPUT);
  pinMode(53,OUTPUT);
  
  // setup analog to digital converter  

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("Hello World!");
  
}

// the loop function runs over and over again forever
void loop() {
 
  val=analogRead(s_pin0);
  Serial.println(val); //debug print
  if (val<512){
    digitalWrite(52,HIGH); //LED 0 cathode high (on)
  } else {
    digitalWrite(52,LOW); //LED 0 cathode low (off)
  }

  
  val=analogRead(s_pin1);
  Serial.println(val); //debug print
  if (val<512){
    digitalWrite(53,HIGH); //LED 0 cathode high (on)
  } else {
    digitalWrite(53,LOW); //LED 0 cathode low (off)
  }
  
}
