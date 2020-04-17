/*
  Analog to Digital Converter - Register Level 
  8 bit resolution - Single channel

  Tennessee Technological University
  Tristan Hill - Jan 12, 2016
   
*/
int val;

// the setup function runs once when you press reset or power the board
void setup() {

  // set up LED for testing
  DDRB = DDRB | B00000001;  // sets Arduino Mega pins 53 as output 


  // setup analog to digital converter  
  ADMUX |= B00100000;
  // Set ADC reference to AVREF (bit7-6,REFS1-0)
  // Left adjust ADC result to allow easy 8 bit reading (bit5,ADLAR)
  // Use AC0 (bit4-0,MUX4-0) 
  
  ADCSRA |= B10000111;
  // Enable ADC (bit7,ADEN)
  // Start A2D Conversions (bit6,ADSC)
  // Dont Enable auto trigger (bit5,ADATE)
  // Clear Interrupt Flag (bit4,ADIF)
  // Enable Interrupts (bit3,ADIE)
  // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz (bit 0-2,ADPS2-0)
  
  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("Hello World!");
  
}

// the loop function runs over and over again forever
void loop() {

  //ADMUX &= ~(1<<MUX0); // change channel to ADC0
  ADCSRA |= (1<<ADSC); //start conversions
  while (!(ADCSRA&B00010000)); // wait for conversion flag
  val=ADCH;
  Serial.println(val); //debug print
  if (val<128){
    PORTB |= B00000001; //LED 0 anode high (on)
  } else {
    PORTB  &= B11111110;// LED 0 anode low (off)
  }
  ADCSRA |= (1<<ADIF); //clear conversion flag

}
