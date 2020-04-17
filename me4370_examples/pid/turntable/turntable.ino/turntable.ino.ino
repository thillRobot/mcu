/*  
 *  ME4370 - Mechatronics - Tennessee Technological University   
 *  Tristan Hill - April 25, 2016
 *  PID turntable 
*/  

// initialize global variables
volatile float cps=00,ref_cps=1000,diff_cps=0,prev_cps=0,err=0,err_prev=0, err_d=0,err_i=0,ctrl_cmd=0;

volatile int8_t p_ctr=0;

//volatile int16_t ctrl_cmd=0;
volatile int16_t counts=0,prev_counts=0; // reference counts per second

volatile bool prev_sign=1,sign_change=0; 

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

ISR(TIMER1_OVF_vect) // timer1 overflow interrupt - control loop
{

    // implement the PID controller
    
    cps=(float)counts/(.0041/2.0);

    diff_cps=cps-prev_cps;
    
    counts=0;

    err=(ref_cps-cps);
    err_d=(err-err_prev)/(.0041/2.0);
    err_i=err_i+err;
    //ctrl_cmd=err*5+err_d*0+err_i*0;
    ctrl_cmd=err*.01+err_i;//*0.0001+err_d*0.001;
    //ctrl_cmd=err*.1+err_i*0.0001+err_d*0.001;
    
    err_prev=err;
    prev_cps=cps;

    // handle 3men motor driver direction latching
    sign_change=0;
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
      delayMicroseconds(500);
      PORTB&=0b11111101; 
      prev_sign=1;
    }
    else if (sign_change && (ctrl_cmd<0))
    {
      PORTB|=0b00000011;
      delayMicroseconds(500);
      PORTB&=0b11111110;
      prev_sign=0;
    }

    // apply control input saturation (10 bit PWM) 
    if (abs(ctrl_cmd)>1024){
      ctrl_cmd=1024;
    }
    if (abs(ctrl_cmd)<0){
      ctrl_cmd=0;
    }
    
    // set the control input duty cycle
    OCR4A=(int)(abs(ctrl_cmd)); 
    //Serial.print(ctrl_cmd,DEC);       
}

ISR(TIMER3_OVF_vect) // timer3 overflow interrupt - data printing only
{ 
  if (p_ctr==10){ //only print every 10th execution
    Serial.print("Reference:");
    Serial.print(ref_cps,DEC); 
    Serial.print("\n\n");
    
    Serial.print("Actual:");
    Serial.print(cps,DEC); 
    Serial.print("\n\n");
    Serial.print(abs(ctrl_cmd)); 
    Serial.print("\n\n");
    p_ctr=0; 
    err_i=0; // zero the integral error
  }
  
  p_ctr++;
}


void setup() {
  // setup serial monitior
  Serial.begin(250000);     
  while (! Serial);
  delay(500);
  Serial.println("PID turntable - TTU!");

  cli(); //turn interrupts off during setup
  
  // setup Timer 1 - control loop timer - fast
  TCCR1A |= 0b00000000; 
  TCCR1B |= 0b00000001; //timer prescale = 1 (no prescale)
  TIMSK1 |= 0b00000001; // overflow interrupt enable 

  // setup Timer 3 - data printing timer - slow
  TCCR3A |= 0b00000000; 
  TCCR3B = 0b00000101; //timer prescale = 1024 (no prescale)
  TIMSK3 |= 0b00000001; // overflow interrupt enable 
  
  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  DDRE &= 0b11001111; // PE4 and PE5 as inputs
  EICRB |= 0b00000101;  // logical change trigger on INT4 and INT5 (table 15-3)
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5

  /// 10 bit fast pwm mode, normal port operation on Timer 4
  TCCR4A |= (1<<COM4A1) | (1<<WGM41) | (1<<WGM40);  
  TCCR4B |= (1<<WGM42)  | (1<<CS40);                     
  TCCR4B &= ~(1<<CS41) & ~(1<<CS42);
  OCR4A = 555 ; // this values controls the duty cycle duty(%)=OCR4A/1024*100
  DDRH |= (1<<PH3); //data direction register for PWM pin

  // setup analog to digital converter  
  ADMUX |= 0b00100000;
  // Set ADC reference to AVREF (bit7-6,REFS1-0)
  // Left adjust ADC result to allow easy 8 bit reading (bit5,ADLAR)
  // Use AC0 (bit4-0,MUX4-0) 
  ADCSRA |= 0b10000111;
  // Enable ADC (bit7,ADEN)
  // Start A2D Conversions (bit6,ADSC)
  // Dont Enable auto trigger (bit5,ADATE)
  // Clear Interrupt Flag (bit4,ADIF)
  // Enable Interrupts (bit3,ADIE)
  // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz (bit 0-2,ADPS2-0)

  // direction pins for 3men motor driver
  DDRB |= 0b00000011;
  PORTB|= 0b00000011;
  delay(200);
  PORTB&=0b11111110;

  sei(); //turn interrupts on 

  }


void loop() {

  ADCSRA |= (1<<ADSC); //start conversions
  while (!(ADCSRA&0b00010000)); // wait for conversion flag(ADIF)
  int val=ADCL>>6|ADCH<<2; // IMPORTANT !!! access ADCL before ADCH !!! 

  ref_cps=(float)val*10;
  
  //ref_cps=0;
}



