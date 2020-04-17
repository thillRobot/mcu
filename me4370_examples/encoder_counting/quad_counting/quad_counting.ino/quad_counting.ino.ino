/*  
 *  ME4370 - Tennessee Technological University   
 *  Tristan Hill - March 12, 2016
 *  Quadrature Encoder Counting
*/  

// interrupt counters
volatile uint8_t itp_4=0x00,itp_5=0x00;
volatile int16_t counts=0,cps=0;
volatile float time_sec=0;

// define the ISR functions
ISR(INT4_vect)
{
  bool chA = PINE&=0b00010000;
  bool chB = PINE&=0b00100000;
  if(chA&&(!chB)){
      counts++;
  }
  else if(chA&&chB){
      counts--;
  }
  else if((!chA)&&chB){
      counts++;
  } 
  else if ((!chA)&&(!chB)){
      counts--;
  }  
  TIFR4 |= (1<<ICF4); //clear the flag (not sure this is working) 
}
ISR(INT5_vect)
{
  bool chA = PINE&=0b00010000;
  bool chB = PINE&=0b00100000;
  if(chB&&chA){
      counts++;
  }
  else if(chB&&(!chA)){
      counts--;
  }
  else if((!chB)&&(!chA)){
      counts++;
  } 
  else if ((!chB)&&(chA)){
      counts--;
  }  
  TIFR5 |= (1<<ICF5); //clear the flag (not sure this is working) 
}


ISR(TIMER1_OVF_vect) {
    
    time_sec=time_sec+.0041/2.0; // need to figure this number out! 1/16e6*65536/2
    cps=counts/(.0041/2.0);
    counts=0;

}

void setup() {

  // setup serial monitior
  Serial.begin(9600);     
  while (! Serial);
  delay(100);
  Serial.println("Quadrature Encoder Counter- TTU Mechatronics!");

    // setup Timer 1
  TCCR1A |= 00000000; //timer prescale = 1 (no prescale)
  TCCR1B |= B00000001;
  TIMSK1 |= 0b00000001; // overflow interrupt enable 
  
  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  cli(); //turn interrupts off during setup
  DDRE &= 0b11001111; // PE4 and PE5 as inputs
  EICRB |= 0b00000101;  // logical change trigger on INT4 and INT5 (table 15-3)
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5
  sei(); //turn interrupts on for use
  
}

void loop() {

  delay(1000);
  Serial.print("counts: ");
  Serial.print(counts,DEC); 
  Serial.print("\n\n"); 

  Serial.print("counts per second: ");
  Serial.print(cps,DEC); 
  Serial.print("\n\n");
}
  
