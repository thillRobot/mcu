// Simple 16 bit Timer example 
// Tristan Hill - January,14,2016

int c_time=0;
int c_cnt=0; 
volatile uint16_t ofcnt=0;
volatile float   time_sec=0;


void setup() {
  // put your setup code here, to run once:

  // setup Timer 1
  TCCR1A = 0b00000000; // 
  TCCR1B = 0b00000001; // clock select - timer prescale = 1 (no prescale)
  TIMSK1 = 0b00000001; // overflow interrupt enable 

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("16-bit timer example");

}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(200);
  c_time=TCNT1L;
  
  while (!(TIFR1&B00000001)); //polling to count overflow
  c_cnt++;

  //Serial.println(c_time);
  
  Serial.print("Time: \n");
  Serial.print(time_sec,DEC);
  Serial.print("\n");
}

// ********************************************************************************
// Interrupt Routines
// ********************************************************************************
// timer1 overflow
ISR(TIMER1_OVF_vect) {

    ofcnt++;
    time_sec=time_sec+.004096;// /2; // need to figure this number out! 1/16e6*65536/2
    //time_sec=time_sec+.032768; // need to figure this number out! 1/16e6*65536/2

}

