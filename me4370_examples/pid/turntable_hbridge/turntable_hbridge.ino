/*  
 *  ME4370 - Tennessee Technological University   
 *  Tristan Hill - March 12, 2016
 *  PID turntable 
*/  

// interrupt counters
volatile uint8_t itp_4=0x00,itp_5=0x00;
volatile uint16_t counts=0;
volatile float time_sec=0;
volatile int16_t ctrl_cmd=0;
volatile int16_t ref_cps=50, cps=0; // reference counts per second
// define the ISR functions
ISR(INT4_vect) //channel A interrupt
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
  TIFR4 |= (1<<ICF4); //clear the flag
}
ISR(INT5_vect) //channel B interrupt
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
  TIFR5 |= (1<<ICF5); //clear the flag
}

// timer1 overflow
ISR(TIMER1_OVF_vect) {

    time_sec=time_sec+.0041/2.0; // need to figure this number out! 1/16e6*65536/2
    cps=counts/(.0041/2.0);
    counts=0;

    ctrl_cmd=(ref_cps-cps)*5;
    //ctrl_cmd=150;
    
    if( ctrl_cmd<0){
      PORTB=0b00000011;
      delay(3);
      PORTB=0b00000010;
    }
    else
    {
      PORTB=0b00000011;
      delay(3);
      PORTB=0b00000001;
    }

    if (abs(ctrl_cmd)>250){
      ctrl_cmd=250;
    }
    
    if (abs(ctrl_cmd)<200){
      ctrl_cmd=200;
    }
    
    
    
    
    
}

void setup() {

  // setup serial monitior
  Serial.begin(9600);     
  while (! Serial);
  delay(100);
  Serial.println("PID turntable - TTU!");

  // setup motor driver
  pinMode(4, OUTPUT);

  DDRB = 0b11111111;

  // setup Timer 1
  TCCR1A |= 00000000; //timer prescale = 1 (no prescale)
  TCCR1B |= B00000001;
  TIMSK1 |= 0b00000001; // overflow interrupt enable 
  
  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  cli(); //turn interrupts off during setup
  DDRE &= 0b11001111; // PE4 and PE5 as inputs
  EICRB |= 0b00000101;  // logical change trigger on INT4 and INT5 (table 15-3)
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5
  sei(); //turn interrupts on 

  /*
  PORTB=0b00000011;
  delay(1000);
  PORTB|=0b00000001;
  */
  analogWrite(4, 200);

 
}

void loop() {

  /*
  
  analogWrite(4, 100);
  PORTB=0b00000010;
  delay(2000);
  PORTB=0b00000011;
  delay(1);
  PORTB=0b00000001;
  delay(2000);
  PORTB=0b00000011;
  delay(1);
*/

  //analogWrite(4, 200);
  
 // delay(100);
 // Serial.print("counts: ");
 // Serial.print(counts,DEC); 
 // Serial.print("\n\n"); 
 /*
  Serial.print("cps: ");
  Serial.print(cps,DEC); 
  Serial.print("\n"); 

  Serial.print(abs(ctrl_cmd),DEC); 
  Serial.print("\n\n");
*/

  //analogWrite(4, 225);
}
  
