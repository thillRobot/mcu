// PWM output example - 10 bit
// Tristan Hill - January,15,2016
//              - February 26,2019
// interfacing with the 'Sabertooth v1.0 2x12' motor driver
// Mega sends a PWM which is filtered into an analog signal which is read by the sabertooth
// the filter is a basic RC filter with a 10k ohm resistor and a .1 uF capacitor
// the signal ranges from 0 to 5 with a 2.5 v center

// user def funs prototypes here
double map_double(double x, double in_min, double in_max, double out_min, double out_max);

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  // TCCR1A |= B10000011;
  TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)

  // prescale 1 (none)
  //TCCR1B |= (1<<WGM12)  | (1<<CS10);                //CS12:0=001 -> 15.63 kHz
  //TCCR1B &= ~(1<<CS11) & ~(1<<CS12);

  // prescale 8
  TCCR1B |= (1 << WGM12)  | (1 << CS11) ;           //CS12:0=010 -> 1.95 kHz
  TCCR1B &= ~(1 << CS10) & ~(1 << CS12);

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
  //OCR1A = 555 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100


  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output
  DDRB |= (1 << PB5); //data direction register

  // setup serail monitior
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  double T = 1.0 / 61.0 * 1000.0; // period in ms
  double power = 0;
  //double ms_high;
  int pwm_high;

  // map motor power(?) [-100 100] to high time(ms) [1.0 2.0]

  //v_out=map_double(power,-100.0,100.0,1.0,2.0);

  pwm_high = map_double(power, -100, 100, 0, 1023);

  OCR1A = pwm_high;

  Serial.print("Period(ms): ");
  Serial.print(T, DEC);
  Serial.print('\n');
  Serial.print("Power (-100,100): ");
  Serial.print(power, DEC);
  Serial.print('\n');
  Serial.print("PWM Compare value (10bit): ");
  Serial.print(pwm_high, DEC);
  Serial.print('\n');
  Serial.print('\n');

  delay(100);

}

// this fuction maps a double from one range to another
double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
