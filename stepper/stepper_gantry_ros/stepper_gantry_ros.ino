/*
 * 
  Tennessee Technological University
  Tristan Hill and Stephen Canfield - April, 08 2022, June 19, 2025
  
  This is example Code for the Fuyu Motion, XYZ gantry table.
 
  This version uses loops and direct register assignments to control the three axes (x,y,z axis) of the machine.

  ISR based delay functions 'fine_delay' and 'coarse_delay' are used the set the travel speed of the y axis

  Rosserial is used to interface with the ROS(1) for the purpose of synchonizing with the gocator

 Hardware connections:
 x axis (two steppers coupled at port): PB7 - dir, PB6 - step
 y axis; PB5 - dir, PB4 - step
 z axis; PH6 - dir, PH5 - step

 Using stepper drive DM556n - stepper online
 dir chang should be ahead of falling Pulse by 5 microseconds
 Pulse width not less than 2.5 microseconds, triggers on falling edge only
   
*/

#include <ros.h>
#include <std_msgs/String.h>

                // stepper motor and power screw parameters
#define SPR 400 // steps per revolution, (steps/rev)
#define MMPR 10 // mm travel per revolution, (mm/rev)

int dt, cnt;
bool dir;

int c_time=0;
//volatile uint16_t ofcnt=0;
//volatile float   time_sec=0;
uint16_t ofcnt=0;
int prescale=1;
float ofdt;
float oftime=0;

char tmp [50]; // temp var and buffer for serial printing
char buf [100];

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

// the setup function runs once when you press reset or power the board
void setup() {

  DDRB=0b11110000; // set Port B PB7:4 outputs
  DDRH=0b01100000; // set Port H PH6:5 to all outputs
  PORTB = 0b00000000;
  PORTH = 0b00000000;
 
  cnt=0; // counter for loop()
  dir=0; // direction to step
  dt=5;  // time delay between steps (micro seconds)

  // setup Timer 1 (16bit)
  TCCR1A  = 0b00000000; 
  TCCR1B  = 0b00000001; //timer prescale = 1 (no prescale)
  TIMSK1  = 0b00000001; // overflow interrupt enable 
  
  //Serial.begin(115200);

  nh.initNode();
  nh.advertise(chatter);

}

void loop() {

  float dt_check=0;  
  float sum_dt=0;
  float avg_dt;
  
  float travel = 50; // (mm)
  float travel_rate = 10; // (mm/sec)
  float steps_per_mm= SPR/MMPR; //(steps/rev)/(mm/rev)->(steps/mm)
  float step_rate=travel_rate*steps_per_mm; //(mm/sec)*(steps/mm)->(steps/sec)

  int n_steps=travel*steps_per_mm;  // (mm)*(steps/mm)->(steps)

  float hi_time=1/(step_rate/2); // time up in seconds
  float lo_time=1/(step_rate/2); // time up in seconds
  
  sprintf(buf,"dir: %i\n\r",dir);  
  //Serial.print(buf);

  sprintf(buf,"n_steps: %i\n\n\r", n_steps);  
  //Serial.print(buf);
  
  dir=!dir; // toggle the direction 
  cnt=0;

  for(int i=0;i<n_steps;i++){
    
    // set direction bits
    PORTB^=(-dir^PORTB)&(0b00100000); // set direction for PB5

    // toggle the step bits 
    PORTB^=(1<<4); // toggle bit PB4 (up?)
    dt_check=isr_delay(hi_time); // step up for hi_time seconds
    //dt_check=dt_check+fine_delay(.004);

    PORTB^=(1<<4); // toggle bit PB4 (down?)
    dt_check=isr_delay(lo_time)+dt_check; // step down for lo_time seconds
    
    sum_dt=sum_dt+dt_check; // calc total step time for debug

  } 

  avg_dt=sum_dt/n_steps;  // calculate average step time
  
  dtostrf(avg_dt,10,7,tmp); // print for debug after traveling 
  sprintf(buf,"avg_dt: %s\n\r",tmp);  
  //Serial.print(buf);

  dtostrf(get_time(),10,7,tmp);
  sprintf(buf,"curr_time: %s\n\n\r",tmp);  
  //Serial.print(buf);
  
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  isr_delay(hi_time);
  //delay(1000);  
  cnt++;
}


// ********************************************************************************
// Subroutines (functions) defined below
// ********************************************************************************

// function to delay for specfied time using ISR fine timer, work in progress
float fine_delay(float duration){

  float prev_oftime, curr_oftime, prev_time, curr_time, dt;
  uint16_t curr_tcnt, prev_tcnt;

  prev_oftime=oftime;
  prev_tcnt=TCNT1;   
  prev_time=prev_oftime+prev_tcnt*1/16000000.0;

  dt=-1; // force into the loop first time
  while (dt<duration){

    curr_oftime=oftime;    
    curr_tcnt=TCNT1;

    if (curr_tcnt>prev_tcnt){   // catch and fix fine timer roll over issue
                                // possibly polling before the ISR completes at tcnt1 rollover
      curr_time=curr_oftime+curr_tcnt*1/16000000.0;
    }else{
      curr_time=curr_oftime+(curr_tcnt+65535)*1/16000000.0;
    }    
    
    dt=curr_time-prev_time; // possibly polling before the ISR completes at tcnt1 rollover

  }
  
  return dt;

}


// function to delay for specfied time using ISR coarse timer, work in progress
float coarse_delay(float duration){

  float prev_oftime, curr_oftime, prev_time, curr_time, dt;
  prev_oftime=oftime;
  prev_time=prev_oftime;

  dt=-1; // force into the loop first time
  while (dt<duration){

    curr_oftime=oftime;    

    if (curr_oftime>=prev_oftime){   // catch and fix fine timer roll over issue
                                // possibly polling before the ISR completes at tcnt1 rollover
      curr_time=curr_oftime;
    }else{    
      dtostrf(prev_oftime,14,10,tmp); // debug prints should be cleaned up
      sprintf(buf,"prev_oftime: %s\n\n\r",tmp);  
      //Serial.print(buf);
      dtostrf(curr_oftime,14,10,tmp);
      sprintf(buf,"curr_oftime: %s\n\n\r",tmp);  
      //Serial.print(buf);
      curr_time=-99.99;
    }    
    
    dt=curr_time-prev_time; // possibly polling before the ISR completes at tcnt1 rollover

  }
  
  return dt;

}


// function to choose appropriate delay routine, this is a bit of a hack, I dont like it
float isr_delay(float duration){

  if (duration<=.004){
    dt=fine_delay(duration);
  }else{
    dt=coarse_delay(duration);
  }

}
 

// function to calculate current time
float get_time(void) {
 
  return oftime+TCNT1*1.0/16000000.0;

}

// ********************************************************************************
// Interrupt Routines
// ********************************************************************************

// timer1 overflow routine
ISR(TIMER1_OVF_vect) {

  ofcnt++;
  oftime=oftime+1.0/16000000.0*65536*prescale; // 1/16e6*65536

}
