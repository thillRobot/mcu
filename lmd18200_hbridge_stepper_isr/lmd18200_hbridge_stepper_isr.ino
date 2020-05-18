//
// LMD18200 H-BRIDGE Interface for driving a Bi-Polar Stepper Motor
// Tristan Hill - March 06, 2019
// 
// This is very primative and needs work... but it works!
// Next Goal: use ineterupts to control timing of pulse train
//

/**************** Wiring Diagram******************* 

  MEGA2560                 LMD18200 (1)
  ________                 ___________                     

                          pin1(bootstrap input1)<---> 10nF cap to pin2
                          pin2(motor output 1)  <---> stepper motor yellow (mercury motor SM42BYG011-25)
        PA0 (pin22) <---> pin3(direction input) 
        PA1 (pin23) <---> pin4(brake input)
        PA2 (pin24) <---> pin5(PWM input)  
                          pin6(VS power) <--------> 12v
                GND <---> pin7(GND))<-------------> GND 
                          pin8(current sense)
                          pin9(thermal flag)
                          pin10(motor output 2) <---> stepper motor blue
                          pin11(bootstrap input 2) <---> 10nF cap to pin10

                           LMD18200 (2)
                           ___________ 
                            
                          pin1(bootstrap input1)<---> 10nF cap to pin2
                          pin2(motor output 1)  <---> stepper motor red
        PA3 (pin25) <---> pin3(direction input) 
        PA4 (pin26) <---> pin4(brake input)
        PA5 (pin27) <---> pin5(PWM input)  
                          pin6(VS power) <--------> 12v
                GND <---> pin7(GND))<-------------> GND 
                          pin8(current sense)
                          pin9(thermal flag)
                          pin10(motor output 2) <---> stepper motor green
                          pin11(bootstrap input 2) <---> 10nF cap to pin10

*/


// user def funs prototypes here
double map_double(double x, double in_min, double in_max, double out_min, double out_max);

void cmd_fullstep(int steps);
void cmd_halfstep(int steps);

// Setup the output sequences - use PORTA for now
// this looks much different using this lmd18200 chip
// setting the outpout states is counterintuitive. just use the given truth table in the data sheet
// PORTA = 0b00000101;  // Source 1, Sink 2     pin2:high pin10:low     
// PORTA = 0b00000100;  // Source 1, Source 2   pin2:low  pin10:high     
// PORTA = 0b00000001;  // Sink 1, Source 2     pin2:high pin10:high    
// PORTA = 0b00000110;  // Sink1, Sink2         pin2:low  pin10:low
// PORTA = 0b00000001;  // None, None           pin2:-    pin10:-      
  
int fullstep[4]={0b00100100,0b00100101,0b00101101,0b00101100} ;    
// 0b00100100; // low high, low high
// 0b00100101; // low high, high low
// 0b00101101; // high low, high low
// 0b00101100; // high low, low high

int halfstep[8]={0b00100100,0b00001100,0b00101100,0b00101001,0b00101101,0b00001101,0b00100101,0b00100001} ;
// 0b00100100; // low high , low high
// 0b00001100; // none none, low high
// 0b00101100; // high low, low high
// 0b00101001; // high low, none none 
// 0b00101101; // high low, high low
// 0b00001101; // none none, high low
// 0b00100101; // low high, high low
// 0b00100001; // low high, none none  

int seq_cnt=0;
//bool seq_flag=1; //0 - sequence incomplete, 1 - sequence complete 

int step_cnt=0;
int step_cmd=0;

// volatle is crucial here, we need to invetigate this
volatile bool step_flag=1; //0 - steps incomplete, 1 - steps complete 

volatile uint16_t ofcnt=0;
volatile float   time_sec=0;
             
void setup() {
  // put your setup code here, to run once:
  
  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("LMD18200 Driver Setup!");

  // PWM OUTPUT 
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  TCCR1A |= (1<<COM1A1) | (1<<WGM11); //WGM13:0 = 1110
  TCCR1A &= ~(1<<WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 14, bits WGM13:0=1110)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)

  // use prescale options for differnt frequency ranges
  // prescale 1 (none)
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS10);   //CS12:0=001 -> 244.5 Hz          
  //TCCR1B &= ~(1<<CS11) & ~(1<<CS12);                 //     
  // prescale 8
  TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS11) ; //CS12:0=010 -> 30.5 Hz       
  TCCR1B &= ~(1<<CS10) & ~(1<<CS12);                //
   
  // prescale 64
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS11) | (1<<CS10);  //CS12:0=011 -> 3 Hz (not square wave)
  //TCCR1B &= ~(1<<CS12);                               // ->   
  // prescale 256
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS12);                //CS12:0=100 -> ~1Hz (not square wave)             
  //TCCR1B &= ~(1<<CS10) & ~(1<<CS11);  
  // prescale 1024
  //TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS12) | (1<<CS10);      //CS12:0=101 -> ~2Hz (not square wave)            
  //TCCR1B &= ~(1<<CS11); 
  
  // Set compare value to 1Hz at 16MHz AVR clock
  double f_base=30.5; // Hz
  double T_base,T,f;
  long T_pwm;

  // Choose a frequency for the interrupt, start with the 'base' freqwncy and divide to go faster
  // this seems to work, meaured with scope, accurate to around +-1.5Hz @ 600Hz
  f=30;
  
  T_base=1.0/f_base;
  T=1.0/f;
  T_pwm=map_double(T,0,T_base,0,65535);
  
  ICR1=T_pwm; // this register defines TOP in mode 14
  //ICR1=65535;
  // whenever a match occurs OC1A toggles
  OCR1A = 10000; // this values controls the duty cyclee 

  // Use PORTA for control pins (output)
  DDRA=0b11111111;

}

void loop() {

  cmd_fullstep(25);
  Serial.println("returned from stepping");
  delay(500);
  
}


void cmd_fullstep(int steps)
{
   step_cmd=steps; 
   step_cnt=0;
   step_flag=0;
   
  // seq_flag=0;
   
   TIMSK1 |= 0b00000001;  // enable overflow interrupt
   
   //Serial.println("Starting Stepping");
   while(step_flag==0)
   {
      //delay(1);   
      //Serial.println("Stepping"); // i cannot figure out why this wait here matters  
                                    // if I leave it out it gets stuck after first set of steps
                                    // the solution is using a 'volatle bool' for a global, we need to invetigate this    
   }
   //Serial.println("Finished Stepping");
}





// ********************************************************************************
// Interrupt Routines
// ********************************************************************************

// timer1 overflow // step here
ISR(TIMER1_OVF_vect) 
{
    Serial.println("TOV1 ISR");
    
    if((seq_cnt==0)&&(step_cnt==step_cmd)) // finished last sequence
    {
      //seq_flag=1;
      step_flag=1;
      TIMSK1 &= 0b11111110; // turn off interrupt
      //Serial.println("Sequence Finshed");  
    }else if(seq_cnt==0)                  // finished this sequence
    {
      PORTA=fullstep[seq_cnt];
      //seq_flag=1;
      step_cnt++;
    }else                                 // mid sequence  
    {
      PORTA=halfstep[seq_cnt];
    }
        
    seq_cnt=(seq_cnt+1)%8;

}

// this fuction maps a double from one range to another
double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
