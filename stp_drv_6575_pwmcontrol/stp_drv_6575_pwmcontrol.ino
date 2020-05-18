/*
  Analog to Digital Converter - Register Level 
  10 bit resolution - Single channel
  IMPORTANT !!! access ADCL before ADCH !!!
  
  Tennessee Technological University
  Tristan Hill - Jan 12, 2016
   
*/
int val;
int dt;

// the setup function runs once when you press reset or power the board
void setup() {

  // set up LED for testing
  DDRB = DDRB | B00000001;  // sets Arduino Mega pins 53 as output 
  DDRA=0b11111111;
 
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

  // SETUP PWM OUTPUT 
  // timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  // TCCR1A |= B10000011;
  TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)
  
  // prescale 1 (none)
  TCCR1B |= (1<<WGM12)  | (1<<CS10);                //CS12:0=001 -> 15.63 kHz          
  TCCR1B &= ~(1<<CS11) & ~(1<<CS12);
    // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR1A = 555 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  
  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output 
  DDRB |= (1<<PB5); //data direction register
  
  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("Hello World!");
  Serial.println("STP-DRV-6575 with potentiometer control");

  dt=20;
}

// the loop function runs over and over again forever
void loop() {

  //ADMUX &= ~(1<<MUX0); // change channel to ADC0
  ADCSRA |= (1<<ADSC); //start conversions
  while (!(ADCSRA&B00010000)); // wait for conversion flag(ADIF)
  val=ADCL>>6|ADCH<<2; // IMPORTANT !!! access ADCL before ADCH !!! 

  Serial.println(val); //debug print
  if (val<128){
    PORTB |= B00000001; //LED 0 anode high (on)
  } else {
    PORTB  &= B11111110;// LED 0 anode low (off)
  }
  ADCSRA |= (1<<ADIF); //clear conversion flag

  dt=val/10.0;
  
  PORTA = 0b00001001;               
  delay(dt);                       
  PORTA = 0b00001000;  
  delay(dt); 
  

}
