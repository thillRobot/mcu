/*
  Analog to Digital Converter - Register Level 

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016
   
*/
//#include <avr/io.h>

double val;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  // pinMode(13, OUTPUT);
  DDRB = DDRB | 00000001;  // sets Arduino pins 2 to 3 as outputs 
  //DDRE = B00000000;

  // setup analog to digital converter 
  //ADMUX |= (1 << REFS0); // Set ADC reference to AVCC (used ext AREF instead)
  //ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading
  // No MUX values needed to be changed to use ADC0
  
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz
  //ADCSRA |= (1 << ADFR);  // Set ADC to Free-Running Mode (ADFR not defined in this scope, use ADATE instead)
  ADCSRA |= (1 << ADATE); // Enable auto trigger
  ADCSRA |= (1 << ADEN);  // Enable ADC
  ADCSRA |= (1 << ADSC);  // Start A2D Conversions
  
  // select auto trigger mode
  //ADCSRB |=(1<<ADTS2)|(1<<ADTS1)|(1<<ADTS0) 

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  
}

// the loop function runs over and over again forever
void loop() {

  
  PORTB=PORTB|B00000001;

  val=ADCH*256+ADCL;

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
