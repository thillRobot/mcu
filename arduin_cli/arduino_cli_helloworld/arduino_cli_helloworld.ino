/*
  
  Tristan Hill - April 26, 2021 - April 27, 2021

  Arduino-CLI (Command Line Interface)
  This is a basic example of using the arduino-cli that shows compile and upload.
  Hello World using the serial connection and minicom

  Check for a connected board:
  $ arduino-cli board list

  Compile the script:
  $ arduino-cli compile --fqbn arduino:samd:mkrwifi1010 arduino_cli_helloworld.ino 

  Upload the script to the board:
  $ arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:samd:mkrwifi1010 arduino_cli_helloworld.ino

  Listen to the serial connection:
  $ minicom ACM0      ('ACM0' refers to a custum config file made with minicom)

*/

void setup() {
  // put your setup code here, to run once:

   // setup serial monitior
  Serial.begin(115200);          //  setup serial
  while (!Serial);
  Serial.println("Hello World Setup!");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello World from Arduino-CLI!"); //debug print
  delay(500); 
}
