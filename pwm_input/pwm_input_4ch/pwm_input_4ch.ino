// Simple 16 bit Timer example
// Tristan Hill - January,10,2016 - March 06, 2019
// Revised - March 19, 2019 
// Revised - March 20, 2019
// goal: measure in RC PWM signals as accuraelty as possible - DONE but not perfect (+-5us)
// goal: add functionality for multilpe channels

int c_time = 0;
int c_cnt = 0;
volatile uint16_t of1_cnt = 0;
volatile uint16_t of3_cnt = 0;

volatile float   time1_sec = 0;
volatile float   time3_sec = 0;
volatile uint8_t itp_0 = 0x00, itp_1 = 0x00;

volatile uint16_t capt_time0, start_time0, hi_time0; // // high time measured in 'microseconds', TCNT1 cycles = 16e6*prescale
volatile uint16_t capt_time1, start_time1, hi_time1;                                      
volatile uint16_t capt_time2, start_time2, hi_time2;
volatile uint16_t capt_time3, start_time3, hi_time3;

int prescale1, prescale3;

char out_str0[50];
char out_str1[50];

void setup() {

  // setup 16bit Timer 1 (global timer) in mode 0
  TCCR1A  = 0b00000000;
  TCCR1B  = 0b00000001; //timer prescale = 8
  TIMSK1  = 0b00000001; // overflow interrupt enable
  prescale1 = 1;

  // setup 16bit Timer 3 (debug print timer) in mode 0
  TCCR3A  = 0b00000000;
  TCCR3B  = 0b00000011;//timer prescale = 64
  //TCCR3B  = 0b00000100;//timer prescale = 256
  TIMSK3  = 0b00000001; // overflow interrupt enable
  prescale3 = 256;

  //setup external interrupts, INT0(PD0,pin21) and INT1(PD1,pin20)
  cli(); //turn interrupts off during setup
  DDRD &=  0b11110000; // PORTE0-3 as inputs
  
  EICRA = 0b01010101;  // logical change trigger on INT0 and INT1 (table 11-3)
  //EICRB |= 0b00001111; // used for ISR 5-7
  EIMSK |= 0b00001111; //external interrupt request enable on INT0 and INT1
  sei(); //turn interrupts on for use

  // setup serial monitior
  Serial.begin(115200);          //  setup serial
  while (!Serial);
  Serial.println("2 channel PWM input example");

  delay(100);
  
}

void loop() {

  delay(100);

}

// ********************************************************************************
// Interrupt Routines
// ********************************************************************************

// timer1 overflow
ISR(TIMER1_OVF_vect) {
  //of1_cnt++;
  //time1_sec = time1_sec + 1.0 / 16000000.0 * 65536 * prescale1; // 1/16e6*65536=.00409 sec
}

// timer3 overflow - this is a slow timer used for debug printing
ISR(TIMER3_OVF_vect) {
  //of3_cnt++;
  //time3_sec = time3_sec + 1.0 / 16000000.0 * 65536 * prescale3;
  
  sprintf(out_str0, "| CH0: %i - CH1: %i  |\n", hi_time0, hi_time1);
  sprintf(out_str1, "| CH2: %i - CH3: %i  |\n", hi_time2, hi_time3);
  
  Serial.print("**************************\n");
  Serial.print("| HI-TIME (microseconds) |\n");
  Serial.print(out_str0);
  Serial.print(out_str1);
  Serial.print("**************************\n");
  
}

// define the ISR functions for external interrupt 0
ISR(INT0_vect) {
  capt_time0=TCNT1;
  if (PIND&=0b00000001) {  
    start_time0 = capt_time0;
  } else {
    hi_time0 = (capt_time0 - start_time0) * 0.0625 * prescale1; // 0.0625=1/16e6*1e6*prescale
  }
}

// define the ISR functions for external interrupt 1
ISR(INT1_vect) {
  capt_time1=TCNT1;
  if (PIND&=0b00000010) {
    start_time1 = capt_time1;
  } else {
    hi_time1= (capt_time1 - start_time1) * 0.0625 * prescale1;
  }
}

// define the ISR functions for external interrupt 2
ISR(INT2_vect) {
  capt_time2=TCNT1;
  if (PIND&=0b00000100) {
    start_time2 = capt_time2;
  } else {
    hi_time2= (capt_time2 - start_time2) * 0.0625 * prescale1;
  }
}

// define the ISR functions for external interrupt 3
ISR(INT3_vect) {
  capt_time3=TCNT1;
  if (PIND&=0b00001000) {
    start_time3 = capt_time3;
  } else {
    hi_time3= (capt_time3 - start_time3) * 0.0625 * prescale1;
  }
}
