// Simple 16 bit Timer example 
// Tristan Hill - January,14,2016

int c_time=0;
int c_cnt=0; 

void setup() {
  // put your setup code here, to run once:

  //DDRB = (1 << PB5);  // OC1A output is on PB5
  
  // PWM OUTPUT 
  // Set up timer to toggle OC1A on match of OCR1A in CTC mode
  //TCCR1A |= (1 << COM1A0);
  //TCCR1A |= B01000000;

  TCCR1A |= 00000000;
  
  // set up timer with prescaler = 1024 and CTC mode 
  //TCCR1B |= (1 << WGM12) | (1 << CS12) | (1 << CS10);

  //TCCR1B |= B00001101;      
  TCCR1B |= B00000001;
  
  // Set CTC compare value to 1Hz at 16MHz AVR clock, with a prescaler of 1024 
  // whenever a match occurs OC1A toggles

  //OCR1A = 15624;

  //DDRB |= B00100000;

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("16-bit timer example");

}

void loop() {
  // put your main code here, to run repeatedly:
  
  //delay(100);
  c_time=TCNT1L;
  
  while (!(TIFR1&B00000001));
  c_cnt++;

  Serial.println(c_cnt);
  
  
}
