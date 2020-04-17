// PWM output example 
// Tristan Hill - January,14,2016

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT 
  // Set up timer to toggle OC1A on match of OCR1A in Fast PWM mode
  TCCR1A |= B10000001; // 8bit fast pwm mode, normal port operation
  
  // set up timer with prescaler = 1 and Fast PWM mode
  //TCCR1B |= B00001001;      

  // set up timer with prescaler = 2 and Fast PWM mode
  TCCR1B |= B00001011;     
  
  // Set CTC compare value to 1Hz at 16MHz AVR clock
  // whenever a match occurs OC1A toggles
  OCR1A = 150; // this values controls the duty cycle, duty(%)=OCRIA/255*100
  
  DDRB |= B11111111;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  
}
