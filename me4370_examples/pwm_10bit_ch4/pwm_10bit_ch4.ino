// PWM output example - 10 bit
// Tristan Hill - January,15,2016

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT 
  // Set up timer to toggle OC1A (PB5, pin6) on match of OCR4A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  TCCR4A |= (1<<COM4A1) | (1<<WGM41) | (1<<WGM40);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)
  
  // prescale 1 (none)
  TCCR4B |= (1<<WGM42)  | (1<<CS40);                //CS12:0=001 -> 15.63 kHz          
  TCCR4B &= ~(1<<CS41) & ~(1<<CS42);
  
  // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR4A = 555 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  
  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output 
  DDRH |= (1<<PH3); //data direction register
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  
}
