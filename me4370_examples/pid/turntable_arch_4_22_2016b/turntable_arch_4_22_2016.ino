/*  
 *  ME4370 - Tennessee Technological University   
 *  Tristan Hill - April 20, 2016
 *  PID turntable 
*/  

// interrupt counters

volatile float cps=0,ref_cps=1000;

volatile int16_t ctrl_cmd=0, err=0,err_prev=0, err_d=0,err_i=0;
volatile int16_t counts=0,prev_counts=0, cps2=0, prev_cps=0, diff_cps=0; // reference counts per second
volatile bool prev_sign=1; // 1-pos  0-neg

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


ISR(TIMER1_OVF_vect) // timer1 overflow interrupt
{
  
    diff_cps=cps-prev_cps;
    
    cps=(float)counts/(.0041/2.0);
    counts=0;
     
    //time_sec=time_sec+.0041/2.0; // need to figure this number out! 1/16e6*65536/2
    
    //cps2=(counts-prev_counts)/(.0041/2.0);
    //prev_counts=counts;
   

    bool sign_change=0;

  
    err=(ref_cps-cps);
    err_d=(err-err_prev)/(.0041/2.0);
    err_i=err_i+err;
    ctrl_cmd=err*.5+err_d*.01+err_i*0.05;
    
    err_prev=err;

    prev_cps=cps;
    //ctrl_cmd=150;
    
    if((ctrl_cmd>0)&&prev_sign)
    {
      sign_change=0;
    }
    else if ((ctrl_cmd>0)&&(!prev_sign))
    {
      sign_change=1;
    }
    else if ((ctrl_cmd<0)&&(!prev_sign)) 
    {
      sign_change=0;
    }
    if ((ctrl_cmd<0)&&prev_sign)
    { 
      sign_change=1;
    }

    if (sign_change && (ctrl_cmd>0))
    {
      PORTB|=0b00000011;
      delay(1);
      PORTB&=0b11111101; 
      prev_sign=1;
    }
    else if (sign_change && (ctrl_cmd<0))
    {
      PORTB|=0b00000011;
      delay(1);
      PORTB&=0b11111110;
      prev_sign=0;
    }

    if (abs(ctrl_cmd)>1020){
      ctrl_cmd=1020;
    }
    if (abs(ctrl_cmd)<10){
      ctrl_cmd=10;
    }

    OCR4A=(int)abs(ctrl_cmd); // set duty
           
}

void setup() {
  // setup serial monitior
  Serial.begin(250000);     
  while (! Serial);
  delay(500);
  Serial.println("PID turntable - TTU!");

  
  // setup motor driver
  pinMode(4, OUTPUT);


  cli(); //turn interrupts off during setup
  
  // setup Timer 1
  TCCR1A |= 0b00000000; 
  //TCCR1B &=~(1<<CS12)&~(1<<CS11); //timer prescale = 1 (no prescale)
  //TCCR1B |=(1<<CS10);
  TCCR1B |= 0b00000001; //timer prescale = 1 (no prescale)
  
  TIMSK1 |= 0b00000001; // overflow interrupt enable 
  
  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  DDRE &= 0b11001111; // PE4 and PE5 as inputs
  EICRB |= 0b00000101;  // logical change trigger on INT4 and INT5 (table 15-3)
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5

 
  /// 10 bit fast pwm mode, normal port operation on Timer 4
  TCCR4A |= (1<<COM4A1) | (1<<WGM41) | (1<<WGM40);  
  // prescale 1 (none)
  TCCR4B |= (1<<WGM42)  | (1<<CS40);                //CS12:0=001 -> 15.63 kHz          
  TCCR4B &= ~(1<<CS41) & ~(1<<CS42);
  
  // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR4A = 555 ; // this values controls the duty cycle
  
  DDRH |= (1<<PH3); //data direction registeres controls the duty cycle, duty(%)=OCR3A/255*100
  
  
  DDRB = 0b00000011;
  analogWrite(7,0);
  PORTB|=0b00000011;
  delay(1000);
  PORTB&=0b11111110;

  sei(); //turn interrupts on 
}

void loop() {

 // Serial.print("counts: ");
 // Serial.print(counts,DEC); 
 // Serial.print("\n\n"); 
  //Serial.print(cps,DEC);
  /*
  Serial.print("CMD PWM: "); 
  Serial.print(ctrl_cmd);
  Serial.print("\n\n");
  Serial.print("CPS: ");
  Serial.print(cps,DEC); 
  Serial.print("\n\n");
*/

/*
  for(int i=1000;i<20000;i=i+5)
  {
    
    ref_cps=-i;
    Serial.print(cps,DEC); 
    Serial.print("\n\n");
    delay(1);
  
  }
  for(int i=20000;i>1000;i=i-5)
  {
    
    ref_cps=-i;
    Serial.print(cps,DEC); 
    Serial.print("\n\n");
    delay(1);
  }
*/

  Serial.print(cps,DEC); 
  Serial.print("\n\n");
  //ref_cps=2000;
  delay(1000);
// ref_cps=-5000;
  
}



