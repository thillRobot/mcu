// Simple 16 bit Timer example
// Tristan Hill - January,10,2016 - March 06, 2019
// Revised - March 19, 2019

// goal measure in RC PWM signals as accuraelty as possible

int c_time = 0;
int c_cnt = 0;
volatile uint16_t ofcnt = 0;
volatile float   time_sec = 0;
volatile uint8_t itp_0 = 0x00, itp_1 = 0x00;

volatile bool hi_flag0 = 0;
volatile uint16_t hi_start0, hi_time0;
volatile bool hi_flag1 = 0;
volatile uint16_t hi_start1, hi_time1;

int prescale = 1;

void setup() {
  // put your setup code here, to run once:

  // setup Timer 1 (global time)
  TCCR1A  = 0b00000000;
  TCCR1B  = 0b00000010; //timer prescale = 1 (no prescale)
  TIMSK1  = 0b00000001; // overflow interrupt enable
  prescale = 8;

  //setup external interrupts, INT0(PE0,pin21) and INT1(PE1,pin20)
  cli(); //turn interrupts off during setup
  DDRE &= 0b11110000; // PE0 and 1 as inputs
  EICRA |= 0b00000101;  // logical change trigger on INT0 and INT1 (table 11-3)
  //EICRB |= 0b00001111; // used for ISR 5-7
  EIMSK |= 0b00000011; //external interrupt request enable on INT0 and INT1
  sei(); //turn interrupts on for use


  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (!Serial);
  Serial.println("16-bit timer example");

  delay(100);
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(100);
  c_time = TCNT1L;

  while (!(TIFR1 & 0b00000001)); //polling to count overflow
  c_cnt++;

  //Serial.println(c_time);

  //Serial.print("Time: \n");
  Serial.println(hi_time0);
  Serial.println(hi_time1);
 // float hi_time0_sec=(float)hi_time0*1.0/16000000.0*prescale;
  //Serial.println(hi_time0_sec);
  //Serial.println("test");
}

// ********************************************************************************
// Interrupt Routines
// ********************************************************************************

// timer1 overflow
ISR(TIMER1_OVF_vect) {

  ofcnt++;
  time_sec = time_sec + 1.0 / 16000000.0 * 65536 * prescale; // 1/16e6*65536=.0001 sec
}

// define the ISR functions for input capture
ISR(INT0_vect) {
  if (hi_flag0 == 0) {
    hi_start0 = TCNT1;
    hi_flag0 = 1;
  } else {
    hi_time0 = TCNT1 - hi_start0;
    hi_flag0 = 0;
  }
}

// define the ISR functions for input capture
ISR(INT1_vect) {
  if (hi_flag1 == 0) {
    hi_start1 = TCNT1;
    hi_flag1 = 1;
  } else {
    hi_time1 = TCNT1 - hi_start1;
    hi_flag1 = 0;
  }
}
