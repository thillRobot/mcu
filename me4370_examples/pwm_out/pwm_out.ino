// PWM output example 
// Tristan Hill - January,14,2016
// Code example from 'larryvc' - avrfreaks.net


void setup() {
  // put your setup code here, to run once:

  //DDRB = (1 << PB5);  // OC1A output is on PB5
  

  // PWM OUTPUT 
  // Set up timer to toggle OC1A on match of OCR1A in CTC mode
  //TCCR1A |= (1 << COM1A0);
  TCCR1A |= B10000001; // 8bit fast pwm mode, normal port operation
  
  // set up timer with prescaler = 1024 and CTC mode 
  //TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);

  TCCR1B |= B00001001;      
  
  // Set CTC compare value to 1Hz at 16MHz AVR clock, with a prescaler of 1024 
  // whenever a match occurs OC1A toggles

  OCR1A = 50;
  
  DDRB |= B11111111;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  
}
