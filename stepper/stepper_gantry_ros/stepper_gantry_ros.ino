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
//#include "ros.h" // use local copy
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

                // stepper motor and power screw parameters
#define SPR 400 // steps per revolution, (steps/rev)
#define MMPR 10 // mm travel per revolution, (mm/rev)

int dt, cnt;
//bool dir; 
int dir;

int c_time=0;
//volatile uint16_t ofcnt=0;
//volatile float   time_sec=0;
int prescale=1;
float ofdt;

uint16_t ofcnt_x=0;
float oftime_x=0;
float step_time_x, hi_time_x, lo_time_x;
int step_cnt_x; 

uint16_t ofcnt_y=0;
float oftime_y=0;
float step_time_y, hi_time_y, lo_time_y;
int step_cnt_y; 

uint16_t ofcnt_z=0;
float oftime_z=0;
float step_time_z, hi_time_z, lo_time_z;
int step_cnt_z; 

char tmp [50]; // temp var and buffer for serial printing
char buf [100];

bool motors_enabled=false;

// function prototypes
float get_time(void);

void enable_motors(char *axis);

void disable_motors(char *axis);

// setup arduino rosserial on hardware serial2 (tx2-pin16, rx2-pin17)
class NewHardware : public ArduinoHardware
{ 
  public:
  NewHardware():ArduinoHardware(&Serial2, 500000){};
};

// instantiate node handle on specified serial port
ros::NodeHandle_<NewHardware,5,5,1024,1024>  nh; 

// setup subscriber with callback function
void messageCb( const std_msgs::Empty& enable_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
 
  if (!motors_enabled){ // enable the motors
    enable_motors("X");
  }else{                // disable the motors
    disable_motors("X");
  } 
  motors_enabled=!motors_enabled; // toggle the global flag
}

ros::Subscriber<std_msgs::Empty> sub("enable_motors", &messageCb );

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

// the setup function runs once when you press reset or power the board
void setup() {

  motors_enabled=false;  
  DDRB=0b00000000; // set Port B PB7:4 outputs
  DDRH=0b00000000; // set Port H PH6:5 to all outputs
  PORTB = 0b00000000;
  PORTH = 0b00000000;
 
  cnt=0; // counter for loop()
  dir=1; // direction to step
  dt=5;  // time delay between steps (micro seconds)

  // setup Timer 1 (16bit), X axis steppers in ISR1
  TCCR1A  = 0b00000000; 
  TCCR1B  = 0b00000001; //timer prescale = 1 (no prescale)
  TIMSK1  = 0b00000001; // overflow interrupt enable 
  
  // setup Timer 3 (16bit), Y axis stepper in ISR3
  TCCR3A  = 0b00000000; 
  TCCR3B  = 0b00000001; //timer prescale = 1 (no prescale)
  TIMSK3  = 0b00000001; // overflow interrupt enable 
  
  // setup Timer 5 (16bit), Z axis stepper in ISR5
  TCCR5A  = 0b00000000; 
  TCCR5B  = 0b00000001; //timer prescale = 1 (no prescale)
  TIMSK5  = 0b00000001; // overflow interrupt enable 
  
  Serial.begin(115200);

  nh.initNode();
  nh.advertise(chatter);

  // on board led for testing, replace pinmode fn soon
  pinMode(13, OUTPUT);
  nh.subscribe(sub);
  
  // setup portA for limit switches 0,1
  DDRA=0b00000011; // bits 0,1 output for sw voltage, 2-7 inputs for sw detect
  PORTA|=0b11111100; // internal pullups on bits 2-7
  // setup PORTC for limit switche 2

  DDRC=0b00000011; // bits 0,1 output for sw voltage, 2-7 inputs for sw detect
  PORTC|=0b11111100; // internal pullups on bits 2-7

  delay(100);
  
  enable_motors("Y");
  home_axis("Y");
  disable_motors("Y");
  delay(500);

  enable_motors("Y");
  step_axis(20,10,1,"Y");
  disable_motors("Y");
  delay(500);

}

void loop() {
  
  float dt_check, dt_check_lo, dt_check_hi;  
  float sum_dt, avg_dt;
    
  //bool step_dir= 0;
  float travel = 20; // (mm)
  float travel_rate = 10; // (mm/sec)
  float steps_per_mm= SPR/MMPR; //(steps/rev)/(mm/rev)->(steps/mm)
  float step_rate=travel_rate*steps_per_mm; //(mm/sec)*(steps/mm)->(steps/sec)

  int n_steps=travel*steps_per_mm;  // (mm)*(steps/mm)->(steps)

  hi_time_x=1/(step_rate/2); // time up in seconds
  lo_time_x=1/(step_rate/2); // time up in seconds
 
  hi_time_y=1/(step_rate/2); // time up in seconds
  lo_time_y=1/(step_rate/2); // time up in seconds
  
  hi_time_z=1/(step_rate/2); // time up in seconds
  lo_time_z=1/(step_rate/2); // time up in seconds
  
  sprintf(buf,"dir: %i\n\r",dir);  
  Serial.print(buf);

  sprintf(buf,"n_steps: %i\n\n\r", n_steps);  
  Serial.print(buf);
  
  dtostrf(hi_time_x,15,12,tmp);  
  sprintf(buf,"hi_time_x: %s\n\r",tmp);  
  Serial.print(buf);
  
  cnt=0;
  
  //set_step_rate(step_rate, dir, "X");
  //set_step_rate(step_rate, dir, "Y");
  //set_step_rate(step_rate, dir, "Z");
  
  dir=!dir; // toggle the direction 
  
  sum_dt=0;
  for(int i=0;i<n_steps;i++){
    
    sum_dt=sum_dt+dt_check_hi+dt_check_lo;  // this is just a test of doing something in the loop
                         // the stepping is happening in the ISR
    
    if (!(i%100)){                          // only publish to ROS about every 10 steps (10 main loops)

      sprintf(buf,"step i: %i\n\r",i);      // printing is OK, but not too often
      Serial.print(buf);
      Serial.println(PINA,BIN);     
 
     // dtostrf(dt_check_hi,15,12,tmp);  
     // sprintf(buf,"dt_check_hi: %s\n\r",tmp);  
     // Serial.print(buf);
      
     // avg_dt=sum_dt/(float)i;  // calculate average step time
     // dtostrf(dt_check_lo,15,12,tmp); // print for debug after traveling 
     // sprintf(buf,"avg_dt: %s\n\r",tmp);  
     // Serial.print(buf);
      
      str_msg.data = buf;
      chatter.publish( &str_msg );
      nh.spinOnce();
    }
    
    // check the limit switches each time
    if (!(PINA&0b00000100)){
      
      dtostrf(get_time(),10,7,tmp);
      sprintf(buf,"sw0 active at curr_time: %s\n\n\r",tmp);  
      Serial.print(buf);
    
      Serial.println(PINA,BIN);     
    }
    if (!(PINA&0b00001000)){
      
      dtostrf(get_time(),10,7,tmp);
      sprintf(buf,"sw1 active at curr_time: %s\n\n\r",tmp);  
      Serial.print(buf);
    
      Serial.println(PINA,BIN);     
    }
    if (!(PINC&0b00000100)){
      
      dtostrf(get_time(),10,7,tmp);
      sprintf(buf,"sw2 active at curr_time: %s\n\n\r",tmp);  
      Serial.print(buf);
    
      Serial.println(PINC,BIN);     
    }
    
  } 
  avg_dt=sum_dt/n_steps;  // calculate average step time
    
//  dtostrf(avg_dt,10,7,tmp); // print for debug after traveling 
//  sprintf(buf,"\n\ravg_dt: %s\n\r",tmp);  
//  Serial.print(buf);

  dtostrf(get_time(),10,7,tmp);
  sprintf(buf,"curr_time: %s\n\n\r",tmp);  
  Serial.print(buf);
  
 // str_msg.data = hello;
 // chatter.publish( &str_msg );
 // nh.spinOnce();
 //  delay(1000);  
 //cnt++;
}


// ********************************************************************************
// Subroutines (functions) defined below
// ********************************************************************************

// function to home the xyz axes using the limit switches
void home_axis(char* axis){

  //enable_motors(axis);

  set_travel_rate(10, 0, axis);

  //while(PINA&0b00000100);
  while(PINC&0b00000100);
    
  //step_axis(100,10,1,axis);
  
  //disable_motors(axis); 

}

// function to move to axes by defined number of steps

void step_axis(int steps, float travel_rate, int direction, char *axis){

  set_travel_rate(travel_rate, direction, axis);
 
  /* 
  if (strcmp(axis,"X")==0){
    step_cnt_x=0;
    while (step_cnt_x<steps){
      //sprintf(buf,"step_cnt: %i\n\r",step_cnt_x);  
      //Serial.print(buf);
    }
  }*/
  
  if (strcmp(axis,"Y")==0){
    step_cnt_y=0;
    while (step_cnt_y<steps){
      sprintf(buf,"step_cnt_y: %i\n\r",step_cnt_y);  
      Serial.print(buf);
    }
  }
  /*
  if (!strcmp(axis,"Z")==0){
    step_cnt_z=0;
    while (step_cnt_z<steps){
      //sprintf(buf,"step_cnt: %i\n\r",step_cnt_x);  
      //Serial.print(buf);
    }
  }*/

//  while (step_cnt_x<steps){
//    sprintf(buf,"step_cnt: %i\n\r",step_cnt_x);  
//    Serial.print(buf);
// }
  
 // disable_motors(axis);
}


// function to set the travel rate of a given axis
void set_travel_rate(float travel_rate, int direction, char *axis){
 
  // float travel_rate = 20; // (mm/sec)
  float steps_per_mm= SPR/MMPR; //(steps/rev)/(mm/rev)->(steps/mm)
  float step_rate=travel_rate*steps_per_mm; //(mm/sec)*(steps/mm)->(steps/sec)

  set_step_rate(step_rate, direction, axis);
}


// function to set the step rate and direction of a given axis, work in progress
void set_step_rate(float step_rate, int direction, char *axis){

  //enable_motors(axis);
  
  bool tmp;
  
  if (direction>0){ // this is a bit of a hack, casting signed int to bool for direction bit
    tmp=true;       // there is a probably a cleaner way but this will work for now
  }else{
    tmp=false;
  }

  if (strcmp(axis,"X")==0){  
    PORTH^=(-tmp^PORTH)&(0b01010000); // set X axis direction with PH6,PH4 
    step_time_x=0; // start the step timer 
  }

  if (strcmp(axis,"Y")==0){  
    PORTB^=(-tmp^PORTB)&(0b00100000); // set Y axis direction with PB7 
    step_time_y=0; // start the step timer
  }

  if (strcmp(axis,"Z")==0){  
    PORTB^=(-tmp^PORTB)&(0b10000000); // set Z axis direction with PB5 
    step_time_z=0; // start the step timer
  }

  // set the global time values to be used in ISR
  hi_time_x=1/(step_rate/2); // time up in seconds
  lo_time_x=1/(step_rate/2); // time up in seconds

  hi_time_y=1/(step_rate/2); // time up in seconds
  lo_time_y=1/(step_rate/2); // time up in seconds

  hi_time_z=1/(step_rate/2); // time up in seconds
  lo_time_z=1/(step_rate/2); // time up in seconds
}


void enable_motors(char *axis){

 //if (strcmp(axis,"X")!=0){
 //   DDRH |= 0b01111000;
 //}
 if (strcmp(axis,"Y")==0){
    DDRB |= 0b00110000; // set Port B PB7:4 outputs
 }
 if (strcmp(axis,"Z")==0){
    DDRB |= 0b11000000; // set Port B PB7:4 outputs
 }
 
  //DDRH=0b11111000; // set Port H PH6:5 to all outputs
  //PORTB = 0b00000000;
  //PORTH = 0b00000000;

}

void disable_motors(char *axis){

  //if (!strcmp(axis,"X")){
  //  DDRH &= 0b10000111;
  // }
  /*
  if (strcmp(axis,"Y")!=0){
    //DDRB &= 0b00111111;
    DDRB = 0b00111111;
  }
  if (strcmp(axis,"Z")!=0){
    DDRB &= 0b11001111;
  }*/
  DDRB=0b00000000; // set Port B PB7:4 inputs
  DDRH=0b00000000; // set Port H PH6:5 to all inputs
}



// function to delay for specfied time using ISR fine timer, work in progress
float fine_delay(float duration){

  float prev_oftime, curr_oftime, prev_time, curr_time, dt;
  uint16_t curr_tcnt, prev_tcnt;

  prev_oftime=oftime_x;
  prev_tcnt=TCNT1;   
  prev_time=prev_oftime+prev_tcnt*1/16000000.0;

  dt=-1; // force into the loop first time
  while (dt<duration){

    curr_oftime=oftime_x;    
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
  prev_oftime=oftime_x;
  prev_time=prev_oftime;

  dt=-1; // force into the loop first time
  while (dt<duration){

    curr_oftime=oftime_x;    

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
 

// functiostepper_gantryn to calculate current time
float get_time(void) {
 
  return oftime_x+TCNT1*1.0/16000000.0;

}

// ********************************************************************************
// Interrupt Routines
// ********************************************************************************


// timer1 overflow routine, used for general clock and step timer X (tmp Y)

ISR(TIMER1_OVF_vect) {

  ofcnt_x++; // increment the overflow counter
  oftime_x=oftime_x+1.0/16000000.0*65536*prescale; // 1/16e6*65536

  step_time_x=step_time_x+1.0/16000000.0*65536*prescale;
  if (step_time_x<=hi_time_x){
    //PORTH|=(1<<5)|(1<<3);
    PORTH|=0b00101000; // bit 4 hi (bit 6 is z axis)
  }else if(step_time_x<=hi_time_x+lo_time_x){
    //PORTH&=~((1<<5)|(1<<3));
    PORTH&=0b11010111; // bit 4 lo
  }else{
    step_time_x=0; // reset timer for next step
    step_cnt_x++; // increment the global step counter for x axis
  }

}

// timer3 overflow routine, used for step timer Y
ISR(TIMER3_OVF_vect) {

  ofcnt_y++; // increment the overflow counter
  oftime_y=oftime_y+1.0/16000000.0*65536*prescale; // 1/16e6*65536

  step_time_y=step_time_y+1.0/16000000.0*65536*prescale;
  if (step_time_y<=hi_time_y){
    PORTB|=0b00010000; // bit 4 hi
  }else if(step_time_y<=hi_time_y+lo_time_y){
    PORTB&=0b11101111; // bit 4 lo
  }else{
    step_time_y=0; // reset timer for next step
    step_cnt_y++; // increment the global step counter for x axis
  }

}


// timer5 overflow routine, used for step timer Z
ISR(TIMER5_OVF_vect) {

  ofcnt_z++; // increment the overflow counter
  oftime_z=oftime_z+1.0/16000000.0*65536*prescale; // 1/16e6*65536

  step_time_z=step_time_z+1.0/16000000.0*65536*prescale;
  if (step_time_z<=hi_time_z){
    PORTB|=0b01000000; // bit 4 hi (bit 6 is z axis)
  }else if(step_time_z<=hi_time_z+lo_time_z){
    PORTB&=0b10111111; // bit 4 lo
  }else{
    step_time_z=0; // reset timer for next step
    step_cnt_z++; // increment the global step counter for x axis
  }

}
