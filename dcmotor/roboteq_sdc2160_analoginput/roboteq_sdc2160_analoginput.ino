// Tennessee Technological University
// Tristan Hill - Jan 12, 2016 , updated May 05, 2020

// This is an example using the Roboteq SDC2160 Brushed DC Motor Controller with the MEGA2560.

// An analog voltage from a potentiometer is converted to a 10 bit value with ADC.
// The value is copied directly to a PWM duty value and the PWM
// signal is output directly to the analog input (0-5v) on the SDC2160 motor controller.
// This is very crude but it works suprisingly well.

int val;
void setup() {
  // put your setup code here, to run once:
    
  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (!Serial);
  Serial.println("Hello World Setup!");

  // PWM OUTPUT 
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode
  // also add  OC1B (PB6, pin12) on match of OCR1B in Fast PWM mode
  
  // 10 bit fast pwm mode, normal port operation
  // TCCR1A |= B10000011;
  //TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10);
  TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10) | (1<<COM1B1);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)
  
  // prescale 1 (none)
  TCCR1B |= (1<<WGM12)  | (1<<CS10);                //CS12:0=001 -> 15.63 kHz          
  TCCR1B &= ~(1<<CS11) & ~(1<<CS12);
  
  // prescale 8
  //TCCR1B |= (1<<WGM12)  | (1<<CS11) ;               //CS12:0=010 -> 1.95 kHz       
  //TCCR1B &= ~(1<<CS10) & ~(1<<CS12);

  // prescale 64
  //TCCR1B |= (1<<WGM12)  | (1<<CS11) | (1<<CS10);    //CS12:0=011 -> 244.1 Hz
  //TCCR1B &= ~(1<<CS12);            
  
  // prescale 256
  //TCCR1B |= (1<<WGM12)  | (1<<CS12);                //CS12:0=100 -> 61.0 Hz            
  //TCCR1B &= ~(1<<CS10) & ~(1<<CS11); 
  
  // prescale 1024
  //TCCR1B |= (1<<WGM12)  | (1<<CS12) | (1<<CS10);      //CS12:0=101 -> 15.3 Hz            
  //TCCR1B &= ~(1<<CS11); 
  
  // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR1A = 555 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  OCR1B = 400 ; // 0-300 is reverse       // I am running a PWM -> Analog Input directly
                // 300-400 is the center  // the deadband is adjustable in Roborun (roboteq)
                // 400-700 is forward    // these are very approximate  
                // past that is dead for some reason 
  
  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output 
  DDRB |= (1<<PB5) | (1<<PB6); //data direction register

  // setup analog to digital converter  
  ADMUX |= B01100000;
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
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(500);
  
  //ADMUX &= ~(1<<MUX0); // change channel to ADC0
  ADCSRA |= (1<<ADSC); //start conversions
  while (!(ADCSRA&B00010000)); // wait for conversion flag(ADIF)
  val=ADCL>>6|ADCH<<2; // IMPORTANT !!! access ADCL before ADCH !!! 
  Serial.println(val);
  OCR1B = val ; // 0-300
}
