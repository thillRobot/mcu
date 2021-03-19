/*
  Tennessee Technological University - ME4370
  Tristan Hill - Jan 11, 2016

  The Serial Monitor - Function Call Level 
  -> allows debugging print to the desktop via USB
   
*/

void setup() {
  // put your setup code here, to run once:

   // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("Hello World Setup!");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello World Loop"); //debug print
  delay(500); 
}
