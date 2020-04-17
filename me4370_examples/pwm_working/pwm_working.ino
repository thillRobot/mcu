// PWM output example - 10 bit
// Tristan Hill - January,15,2016

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT 
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  // TCCR1A |= B10000011;
  TCCR1A |= (1<<COM1A1) | (1<<WGM11); //WGM13:0 = 1110
  TCCR1A &= ~(1<<WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 14, bits WGM13:0=1110)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)
  
  // prescale 1 (none)
  TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS10);   //CS12:0=001 -> 244.5 Hz          
  TCCR1B &= ~(1<<CS11) & ~(1<<CS12);                 // 
  
  // prescale 8
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS11) ; //CS12:0=010 -> 30.5 Hz       
  //TCCR1B &= ~(1<<CS10) & ~(1<<CS12);                // 

  // prescale 64
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS11) | (1<<CS10);  //CS12:0=011 -> 3 Hz (not square wave)
  //TCCR1B &= ~(1<<CS12);                               // -> 
  
  // prescale 256
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS12);                //CS12:0=100 -> ~1Hz (not square wave)             
  //TCCR1B &= ~(1<<CS10) & ~(1<<CS11); 
  
  // prescale 1024
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS12) | (1<<CS10);      //CS12:0=101 -> ~2Hz (not square wave)            
  //TCCR1B &= ~(1<<CS11); 
  
  // Set compare value to 1Hz at 16MHz AVR clock

  ICR1=65535; // this register defines TOP in mode 14
  // whenever a match occurs OC1A toggles
  OCR1A = 12500; // this values controls the duty cycle
  
  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output 
  DDRB |= (1<<PB5); //data direction register
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  
}
