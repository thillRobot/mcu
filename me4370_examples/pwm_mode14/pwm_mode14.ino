
// PWM output example - mode 14 - choose frequency with ICRn and duty cycle with OCRnA
// Tristan Hill - January,18,2016 - March 06, 2019

// user def funs prototypes here
double map_double(double x, double in_min, double in_max, double out_min, double out_max);

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT 
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
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
  double f_base=244.5; // Hz
  double T_base,T,f;
  long T_pwm;

  // Choose a frequency for the interrupt, start with the 'base' freqwncy and divide to go faster
  // this seems to work, meaured with scope, accurate to around +-1.5Hz @ 600Hz
  f=600;
  
  T_base=1.0/f_base;
  T=1.0/f;
  T_pwm=map_double(T,0,T_base,0,65535);
  
  ICR1=T_pwm; // this register defines TOP in mode 14
  //ICR1=65535;
  // whenever a match occurs OC1A toggles
  OCR1A = 10000; // this values controls the duty cycle
  
  // PB5 (pin 11 on mega) as output 
  DDRB |= (1<<PB5); //data direction register

  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("PWM Mode 14 Setup!");

  Serial.println(T,"DEC");
  Serial.println(T_base,"DEC");
  Serial.println(T_pwm);

  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(100);
  
  
}


// this fuction maps a double from one range to another
double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
