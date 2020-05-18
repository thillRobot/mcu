// PWM output example - 10 bit
// Tristan Hill - January,15,2016
// updated - 

void setup() {
  // put your setup code here, to run once:

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
  OCR1B = 400 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  
  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output 
  DDRB |= (1<<PB5) | (1<<PB6); //data direction register
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  
}
