//
// Example code for 3men motor driver
// TTU - Mechatronics - ME4370
// Tristan Hill - April 27 2016
//

// PWM the ADJ pin on driver (0-5v)
// Bring CW pin on driver low for clockwise
// Bring CCW pin on driver low for counterclockwise  
// main issue is switching direction

void ttu_quasimicro(int ms);

void setup() {
  // put your setup code here, to run once:
  
  // direction pins for 3men motor driver
  DDRB |= 0b00000011;
  PORTB|= 0b00000011; // bring them both high to start
  delay(200);

  // 10 bit fast pwm mode, normal port operation on Timer 4 (mega pin 6 -> adj)
  TCCR4A |= (1<<COM4A1) | (1<<WGM41) | (1<<WGM40);  
  TCCR4B |= (1<<WGM42)  | (1<<CS40);                     
  TCCR4B &= ~(1<<CS41) & ~(1<<CS42);
  OCR4A = 100 ; // this values controls the duty cycle duty(%)=OCR4A/1024*100
  DDRH |= (1<<PH3); //data direction register for PWM pin


}


void loop() {
  // put your main code here, to run repeatedly:
  
  PORTB = 0b00000010; // bring them both high to start
  delay(1000);
  PORTB = 0b00000011; // bring them both high to start
  ttu_quasimicro(1);
  //for (int i=1;i<20000;i++){}
  PORTB = 0b00000001; // bring them both high to start
  delay(1000);
  PORTB = 0b00000011; // bring them both high to start
  //for (int i=1;i<20000;i++){}
  
  ttu_quasimicro(1);
  //delayMicroseconds(200);
  }

void ttu_quasimicro(int ms)
{
  for (uint16_t i=1;i<20000;i++){}
  for (uint16_t i=1;i<20000;i++){}
  for (uint16_t i=1;i<20000;i++){}
  for (uint16_t i=1;i<20000;i++){}
  for (uint16_t i=1;i<20000;i++){}
  for (uint16_t i=1;i<20000;i++){}
}
  
