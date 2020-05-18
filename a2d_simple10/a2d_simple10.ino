/*
  Analog to Digital Converter - Register Level 

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016
   
*/
//#include <avr/io.h>

double val;

// the setup function runs once when you press reset or power the board
void setup() {

  // set up LED for testing
  DDRB = DDRB | 00000001;  // sets Arduino Mega pins 53 as output 

  // setup analog to digital converter 
  
  // ADMUX - No MUX values needed to be changed to use ADC0
  // Set ADC reference to AVCC (used ext AREF instead)
  // Right Adjust ADC result to allow easy 10 bit reading
  
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

  
  PORTB=PORTB|B00000001;

  val=ADCH*256+ADCL; // calculate the 10 bit combined value(tested with 10k pot btwn grnd and 5v with 330 ohm on each side)

  if (val<500){
    PORTB=PORTB&111111110;
  }


  //Serial.println(val);  
  //Serial.println("test2"); 
  //(500);
  
  Serial.print(ADCH,DEC);           
  Serial.print('\n');
  Serial.print(ADCL,DEC);             
  Serial.print('\n');
  Serial.print(val,DEC);             
  Serial.print('\n');
  delay(100);
}
