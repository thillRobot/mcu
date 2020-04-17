/*
 
 Simple 16 bit Timer example
 Tristan Hill - January,10,2016 - March 06, 2019
 Revised - March 19, 2019
 Revised - March 24, 2019


 goal: measure in RC PWM signals as accuraelty as possible
 goal: add functionality for multilpe channels
 add 'input capture' instead of 'external interrupts' - it should be more accurate
  
  CH0 - PORTL0 - ICP0
  CH1 - PORTL1 - ICP1
  
*/

volatile uint16_t of4_cnt = 0;
volatile uint16_t of5_cnt = 0;

volatile float time4_sec = 0;
volatile float time5_sec = 0;

volatile uint16_t cap_time4, start_time4, hi_time4; // high time measured in seconds 'TCNT1 cycles = 16e6*prescale'
volatile uint16_t cap_time5, start_time5, hi_time5;

int prescale3, prescale4, prescale5;

char out_str0[30];


void setup() {

    // setup 16bit Timer 3 (debug print timer) in mode 0
    TCCR3A  = 0b00000000; 
    prescale3 = 64; 
    if (prescale3==1)          TCCR3B  = 0b00000001;
    else if (prescale3==8)     TCCR3B  = 0b00000010;
    else if (prescale3==64)    TCCR3B  = 0b00000011;
    else if (prescale3==256)   TCCR3B  = 0b00000100;
    else if (prescale3==1024)  TCCR3B  = 0b00000101;
    TIMSK3  = 0b00000001; // overflow interrupt enable
    
  //setup input capture interrupts, INT0(PD0,pin21) and INT1(PD1,pin20)
  /*
    The Input Capture unit is easy to use in Normal mode. However, observe that the maximum interval between the
    external events must not exceed the resolution of the counter. If the interval between events are too long, the timer
    overflow interrupt or the prescaler must be used to extend the resolution for the capture unit.
  */
  // use 16 bit Timer 4 in mode 0
  cli(); //turn interrupts off during setup

  TCCR4A  = 0b00000000; // COMnA1 =0, COMnB1= 0 , Normal port operation, OCnA/OCnB/OCnC disconnected
  TCCR4B  = 0b01000010; // |(1<<ICES1); //timer prescale = 8, input capture edge select
  TIMSK4  = 0b00100001; //|(1<<ICIE1); // Input Capture Interrupt Enable, overflow interrupt enable
  
  prescale4 = 8; 
    if (prescale4==1)          TCCR3B  = 0b00000001;
    else if (prescale4==8)     TCCR3B  = 0b00000010;
    else if (prescale4==64)    TCCR3B  = 0b00000011;
    else if (prescale4==256)   TCCR3B  = 0b00000100;
    else if (prescale4=1024)  TCCR3B  = 0b00000101;
    
  TCCR5A  = 0b00000000; // COMnA1 =0, COMnB1= 0 , Normal port operation, OCnA/OCnB/OCnC disconnected
  TCCR5B  = 0b01000010; // |(1<<ICES1); //timer prescale = 8, input capture edge select
  TIMSK5  = 0b00100001; //|(1<<ICIE1); // Input Capture Interrupt Enable, overflow interrupt enable
  
  prescale5 = 8; 
    if (prescale5==1)          TCCR3B  = 0b00000001;
    else if (prescale5==8)     TCCR3B  = 0b00000010;
    else if (prescale5==64)    TCCR3B  = 0b00000011;
    else if (prescale5==256)   TCCR3B  = 0b00000100;
    else if (prescale5==1024)  TCCR3B  = 0b00000101;
  DDRL =    0b11111100; // PORTL0,1 (ICP0,ICP1) as input

  sei(); //turn interrupts on for use

  // setup serial monitior
  Serial.begin(115200);          //  setup serial
  while (!Serial);
  Serial.println("Input Capture Example");

  delay(100);

}

void loop() {

  delay(100);
}

// ********************************************************************************
// Interrupt Routines
// ********************************************************************************
// timer3 overflow - this is a slow timer used for debug printing
ISR(TIMER3_OVF_vect) {
  //of3_cnt++;
  //time3_sec = time3_sec + 1.0 / 16000000.0 * 65536 * prescale3;
  
  sprintf(out_str0, "| CH0: %4i - CH1: %4i  |\n", hi_time4, hi_time5);
  
  Serial.print("**************************\n");
  Serial.print("| HI-TIME (microseconds) |\n");
  Serial.print(out_str0);;
  Serial.print("**************************\n");
  
}
// timer4 overflow ISR
ISR(TIMER4_OVF_vect) {
  of4_cnt++; // not currently using overflow counters but they are being counted
  time4_sec = time4_sec + 1.0 / 16000000.0 * 65536 * prescale4; // 1/16e6*65536=.00409 sec
}
// timer5 overflow ISR
ISR(TIMER5_OVF_vect) {
  of5_cnt++;
  time5_sec = time5_sec + 1.0 / 16000000.0 * 65536 * prescale5; // 1/16e6*65536=.00409 sec
}

// define the ISR functions for input capture
ISR(TIMER4_CAPT_vect) {
  cap_time4 = ICR4; // this contains the value of TCNT4 at the time of the interupt
  if (PINL &= 0b00000001) {
    start_time4 = cap_time4; // record time of rising edge (cnts)
    TCCR4B &= 0b10111111;  // edge select low
  } else {
    hi_time4 = (cap_time4 - start_time4) * 0.0625 * prescale4; // // calulate time of pulse on falling edge
    TCCR4B |= 0b01000000; // edge select back high
  }
}

// define the ISR functions for input capture
ISR(TIMER5_CAPT_vect) {
  cap_time5 = ICR5; // this contains the value of TCNT4 at the time of the interupt
  if (PINL &= 0b00000010) {
    start_time5 = cap_time5; // record time of rising edge (cnts)
    TCCR5B &= 0b10111111;  // edge select low
  } else {
    hi_time5 = (cap_time5 - start_time5) * 0.0625 * prescale5; // // calulate time of pulse on falling edge
    TCCR5B |= 0b01000000; // edge select back high
  }
}
