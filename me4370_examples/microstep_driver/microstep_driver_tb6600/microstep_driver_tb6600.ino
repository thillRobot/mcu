//
// Microstep Driver Tb6600 Interface
// Tristan Hill - March, 04, 2019
// 
// This uses the a while loop and the TOV1 flag to handle the step counting

void set_steps(int steps,int dir);

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)

  // prescale 1 (none)
  //TCCR1B |= (1<<WGM12)  | (1<<CS10);                //CS12:0=001 -> 15.63 kHz, T=63.97 us
  //TCCR1B &= ~(1<<CS11) & ~(1<<CS12);

  // prescale 8
  //TCCR1B |= (1 << WGM12)  | (1 << CS11) ;           //CS12:0=010 -> 1.95 kHz , T=512.8 us
  //TCCR1B &= ~(1 << CS10) & ~(1 << CS12);

  // prescale 64
  TCCR1B |= (1<<WGM12)  | (1<<CS11) | (1<<CS10);    //CS12:0=011 -> 244.1 Hz, T=4098 us
  TCCR1B &= ~(1<<CS12);

  // prescale 256
  //TCCR1B |= (1<<WGM12)  | (1<<CS12);                //CS12:0=100 -> 61.0 Hz ,T=16393 us
  //TCCR1B &= ~(1<<CS10) & ~(1<<CS11);

  // prescale 1024
  //TCCR1B |= (1<<WGM12)  | (1<<CS12) | (1<<CS10);      //CS12:0=101 -> 15.3 Hz ,T=65359 us
  //TCCR1B &= ~(1<<CS11);

  // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR1A = 512 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  // 500/16393*1023 -> 500 us pulse at 61. Hz

  DDRB |= 0b11111111; // PA0,PA1 set as output
  //PORTA |= 0b00000011; // enable and set direction
  DDRA  =  0b111111111; // Port A as output
  PORTA &= 0b111111100; // set enable low ('on') and direction low('cw')
  PORTA |= 0b00000001;  // set enable high('off')
  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("Microstep Driver Setup!");
}

void loop() {
  // put your main code here, to run repeatedly:

  set_steps(400,1); // wait inside 'blocking function' for the motor to step

  delay(200);
  
  set_steps(400,-1);
  
  delay(200);

}

void set_steps(int steps, int dir)
{
  if (dir==1)
  {
    PORTA |= 0b00000010;            // set dir high('ccw')  
  }else
  {
    PORTA &= 0b11111101;            // set dir high('cw') 
  }
  
  PORTA &= 0b11111110;              // set enable low('on')
  for (int k = 0; k < steps; k++) {
    while (!(TIFR1 & 0b00000010));  // wait for flag 
    TIFR1 |= 0b00000010;            // clear the flag
  }
  PORTA |= 0b00000001;              // set enable high('off')
}
