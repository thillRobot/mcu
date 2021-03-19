/*
  Analog to Digital Converter - Register Level Programming

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016
   
*/
//#include <avr/io.h>

int val;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 53(PB0) as an output.
   DDRB = DDRB | 00000001;  // sets Arduino mega pin 53 as output
  
  // setup analog to digital converter 
  
  ADMUX |= B00100000;
  // Set ADC reference to AVCC
  // Left adjust ADC result to allow easy 8 bit reading
  // Use AC0 
 
  ADCSRA |= B11100111;
  // Enable auto trigger
  // Enable ADC
  // Start A2D Conversions
  // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  
}

// the loop function runs over and over again forever
void loop() {

  
  PORTB=PORTB|B00000001;// LED anode high (off)

  // check the high byte (8 bit because Left Adjust)
  if (ADCH<128){
    PORTB=PORTB&111111110; //LED anode low (on)
  }

  // print to the serial monitor
  Serial.println(ADCH);  

}
