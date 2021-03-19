//
// LMD18200 H-BRIDGE Interface for driving a Bi-Polar Stepper Motor
// Tristan Hill - March 06, 2019
// 
// Next Goal: use ineterupts to control timing of pulse train - DONE
// Next Goal: add position and velocity control functions - DONE
// Next Goal: integrate with ROS - DONE
// Next Goal: hmm....

// Goal: integrate with 'rplidar_3d' ROS node - progress

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
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

// user def funs prototypes here
double map_double(double x, double in_min, double in_max, double out_min, double out_max);

/******** DEFINE GLOBAL VARIABLES ********/

// Stepper motor sequences
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
// 0b00100101; // low high, high low% % %% 
x = [T0 T_TDMA Tn 0 500] ;
T=mapminmax(x,1,64);
axis([0 n+1 0 500])

wallw=2
wallh=5

figure(2);hold on 
axis([-wallw L+wallw 0 1])
colormap('hot')
cmap =colormap;

for i=1:(length(T-2))
    if i==1
%         wall.vertices=[x0-wallw y0-wallh/2
%                    x0-wallw y0+wallh/2
%                    x0 y0+wallh/2
%                    x0 y0-wallh/2
%                    x0-wallw y0-wallh/2  ];
%         wall.faces   = 1:length(wall.vertices);
%         wall.facecolor =cmap(round(T0)+1,:);%cmap(round(T(i)),:);
%         patch(wall)
    elseif i==n+2
%         wall.vertices=[L y0-wallh/2
%                    L y0+wallh/2
%                    L+wallw y0+wallh/2
%                    L+wallw y0-wallh/2
%                    L y0-wallh/2  ];
%         wall.faces   = 1:length(wall.vertices);
%         wall.facecolor =cmap(round(Tn)+1,:);
%         patch(wall)
    else
        node.vertices=[x0+dx*(i-2) y0
                   x0+dx*(i-2) y0+h
                   x0+dx*(i-1) y0+h
                   x0+dx*(i-1) y0
                   x0+dx*(i-2) y0  ];
        node.faces   = 1:length(node.vertices);
        node.facecolor =cmap(round(T(i)),:);
        node.edgecolor ='white';
        patch(node)
    end
end

// 0b00100001; // low high, none none  

// Setup the output sequences using PORTA for now
// this looks much different using this lmd18200 chip
// setting the outpout states is counterintuitive, just use the given truth table in the data sheet
// PORTA = 0b00000101;  // Source 1, Sink 2     pin2:high pin10:low     
// PORTA = 0b00000100;  // Source 1, Source 2   pin2:low  pin10:high     
// PORTA = 0b00000001;  // Sink 1, Source 2     pin2:high pin10:high    
// PORTA = 0b00000110;  // Sink1, Sink2         pin2:low  pin10:low
// PORTA = 0b00000001;  // None, None           pin2:-    pin10:-      

//flags and counters for stepper control 
int SPR=50; //steps per revolution

volatile int seq_cnt=0;

volatile int pos_curr=0;
volatile int pos_cmd=0;

volatile int step_cnt=0;
volatile int step_cmd=0;

volatile int vel_cmd=0;
volatile int vel_curr=0;

// flags for ROS communication
bool motor_enable=0;  // turn off the motor to begin
//bool motor_dir=1;     // set the direction
bool motor_status=1;  // 0 - 'not currently stepping' // 1 - 'currently stepping' 
int motor_pos=0;
int motor_vel=0;

// volatle is crucial here, we need to invetigate this
volatile bool step_flag=1; //0 - steps incomplete, 1 - steps complete 
volatile bool step_dir=1; //0 - CCW , 1 - CW

volatile bool vel_mode=0; //0 - vel mode off , 1 - vel mode on

// setup ROS client node
ros::NodeHandle nh;
/******** PUBLISHERS ********/
/* publisher object for motor status */
std_msgs::Bool status_msg;
ros::Publisher status_pub("motor_status", &status_msg);

/* publisher object for motor position */
std_msgs::Int32 pos_msg;
ros::Publisher pos_pub("motor_position", &pos_msg);

/* publisher object for debug data */
std_msgs::Int32 debug_msg;
ros::Publisher debug_pub("debug", &debug_msg);


/******** SUBSCRIBERS ********/
/* Callback function for 'motor enable' */
void enable_cb( const std_msgs::Bool& msg){
  motor_enable=msg.data;   // turn on the motor
}
/* Subscriber object for 'motor enable' */
ros::Subscriber<std_msgs::Bool> sub1("motor_enable", &enable_cb );

/* Callback function for 'motor direction' */
void dir_cb( const std_msgs::Bool& msg){
  step_dir=msg.data;   // set the direction
}
/* Subscriber object for 'motor direction'*/
ros::Subscriber<std_msgs::Bool> sub2("motor_dir", &dir_cb );

/* Callback function for 'motor velocity' */
void vel_cb( const std_msgs::Int32& msg){
  motor_vel=msg.data;   // set the direction
  init_step(motor_vel);
}
/* Subscriber object for 'motor velocity'*/
ros::Subscriber<std_msgs::Int32> sub3("motor_vel", &vel_cb );

/* Callback for 'command steps' */
void cmd_steps_cb( const std_msgs::Int32& msg){
  motor_status=1;
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );

  nh.spinOnce();
  
  command_steps(msg.data,step_dir);  // command the steps

  pos_msg.data = motor_pos;
  pos_pub.publish( &pos_msg );
  
  motor_status=0;
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();
}
/* Subscriber object for 'command steps' */
ros::Subscriber<std_msgs::Int32> sub4("cmd_steps", &cmd_steps_cb );

/* Callback for 'command position' */
void cmd_pos_cb( const std_msgs::Int32& msg){
  motor_status=1;
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();
  
  command_pos(msg.data);  // command the position

  pos_msg.data = motor_pos;
  pos_pub.publish( &pos_msg );
  
  motor_status=0;
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();
}
/* Subscriber object for 'command position' */
ros::Subscriber<std_msgs::Int32> sub5("cmd_pos", &cmd_pos_cb );

/* Callback for 'command velocity' */
void cmd_vel_cb( const std_msgs::Int32& msg){
  
  motor_status=1;
  //vel_mode=1;


  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();

  debug_msg.data = 1;
  debug_pub.publish( &debug_msg );
  
  //Serial.println("Command Velocity Callback");
  vel_cmd=msg.data;  
  command_vel(msg.data,1);  // command the velocity

  //pos_msg.data = motor_pos;
  //pos_pub.publish( &pos_msg );
  
  //motor_status=0; // notice the motor is still on
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();
}
/* Subscriber object for 'command velocity' */
ros::Subscriber<std_msgs::Int32> sub6("cmd_vel", &cmd_vel_cb );
/*
// Callback function for 'desired position'
void dir_cb( const std_msgs::Bool& msg){
  motor_dir=msg.data;   // set the desired pos
}
// Subscriber object for 'motor direction'
ros::Subscriber<std_msgs::Bool> sub2("motor_dir", &dir_cb );
*/
            
void setup() {
  // put your setup code here, to run once:
  
  // setup serial monitior
  //Serial.begin(9600);          //  setup serial
  //while (! Serial);
  //Serial.println("LMD18200 Driver Setup!");
 
  init_step(10);
  
  // setup ROS node
  nh.initNode();
  
  // setup subscribers
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  nh.subscribe(sub5);
  nh.subscribe(sub6);
  delay(.01);
  
  nh.spinOnce();
  delay(.01);
  
  // setup publishers
  nh.advertise(status_pub);
  nh.advertise(pos_pub);
  nh.advertise(debug_pub);

}

void loop() {

  /* test the command functions 
  Serial.println(pos_curr);1
  Serial.println("Command Stepping");
  command_steps(40,0);
  Serial.println("Finished Stepping");
  delay(1000);
  Serial.println(pos_curr);

  Serial.println("Command Position");
  command_pos(0);
  Serial.println("Finished Positioning");
  Serial.println(pos_curr);
  delay(1000);

  Serial.println("Command Velocity");
  command_vel(60,0);
  delay(2000);
  
  command_vel(150,1);
  delay(2000);
  command_vel(0,0);
  Serial.println("Finished Velocitying...");
  Serial.println(pos_curr);

  while(1);
  */
  
  //pos_msg.data = motor_pos;
  pos_msg.data = pos_curr;
  pos_pub.publish( &pos_msg );
  nh.spinOnce();
  
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  
  nh.spinOnce();
  delay(.01);
  
}

void init_step(double vel)
{
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode
  // 10 bit fast pwm mode, normal port operation
  TCCR1A |= (1<<COM1A1) | (1<<WGM11); //WGM13:0 = 1110
  TCCR1A &= ~(1<<WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 14, bits WGM13:0=1110)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)
  
  int speed_range=1; //1-slow, 2-fast 
  double f_base;
  // use prescale options for differnt frequency ranges (trade off is resolution)
  if (speed_range==2) //fast range
  {
    TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS11) ; //CS12:0=010 -> 30.5 Hz (measured w/ oscope)       
    TCCR1B &= ~(1<<CS10) & ~(1<<CS12);
    f_base=30.5; // Hz 
  }else if(speed_range==1) //slow range
  {
    TCCR1B |= (1<<WGM13)  | (1<<WGM12)  | (1<<CS11) | (1<<CS10);  //CS12:0=011 -> 3 Hz (not square wave)
    TCCR1B &= ~(1<<CS12);                               // -> 
    f_base=3.0; // Hz  
  }

  // Set compare value to 1Hz at 16MHz AVR clock
  double T_base,T,f_step,f_seq;
  long T_pwm;

  // set the frequency for the interrupt, start with the 'base' frequency and divide to go faster
  // this seems to work, meaured with scope, accurate to around +-1.5Hz @ 600Hz
  
  f_step=vel*SPR/60; // this is frequency of steps, max for mecury stepper is between 150-200 Hz
  
  f_seq=f_step*8; //this is the frequency that the stepping sequence will be cycled (4- full step, 8- halfstep)  
  T_base=1.0/f_base;
  T=1.0/f_seq;
  T_pwm=map_double(T,0,T_base,0,65535);
  
  ICR1=T_pwm; // this register defines TOP in mode 14
  // ICR1=65535;
  // whenever a match occurs OC1A toggles
  OCR1A = 10000; // this values controls the duty cycle (not needed for this application)

  // Use PORTA for control pins (output)
  DDRA=0b11111111;
}

void command_steps(int steps, bool dir)
{
  vel_mode=0;
  step_dir=dir;
  step_cmd=steps; 
  step_cnt=0;
  step_flag=0;

  if (steps==0)
  {
  }else
  {   
    TIMSK1 |= 0b00000001;  // enable overflow interrupt
    while(step_flag==0){   // wait until steps are complete (blocking)   
      pos_msg.data = pos_curr;
      pos_pub.publish( &pos_msg );
      nh.spinOnce();
      delay(.001);
    }
  }
}

void command_pos(int pos)
{
  vel_mode=0;
  pos_cmd=pos;
  int pos_del;
  pos_del=(pos_cmd-pos_curr);//%SPR;

  // take the shortest path
  if (pos_del>=SPR/2)
  {
    pos_del=-(SPR-pos_del);
  }else if (pos_del<=-SPR/2)
  {
    pos_del=SPR+pos_del;
  }
  // and go backwards when needed
  if (pos_del>0)
  {
    command_steps(pos_del,0);
  }else if(pos_del<0)
  {
    command_steps(abs(pos_del),1);
  }else
  {
    command_steps(0,1);
  }
  
}

// note: if this function is used the motor will not stop 
// until it is used again to command a zero velocity (consider fixing)
void command_vel(double vel,int dir)
{
  //vel_mode=1;
  init_step(vel);
  step_dir=dir;

  if (vel==0)
  {
    TIMSK1 &= 0b11111110; // turn off interrupt   
  }else
  {
    TIMSK1 |= 0b00000001;  // enable overflow interrupt
    
    vel_curr=vel_cmd;
    while (vel_cmd==vel_curr)  // wait here unless there has been a change in the commanded velocity
    {
      //vel_curr=vel_cmd;
  
    }
  }
  
}

/********** Interrupt Routines - The timing for stepping is handled here***************/
// timer1 overflow // step here
ISR(TIMER1_OVF_vect) 
{

  if (step_dir==0)
  {
    PORTA=halfstep[7-seq_cnt]; // step backwards
    motor_pos--;
  }else if(step_dir==1)
  {
    PORTA=halfstep[seq_cnt];   // step forwards
    motor_pos++;
  }
  
  if((seq_cnt==7)&&(step_cnt==step_cmd)&&(vel_mode==0)) // finished last sequence (dont stop in vel mode)
  {
    step_flag=1;
    TIMSK1 &= 0b11111110; // turn off interrupt 
  }else if(seq_cnt==0)                  // finished this sequence
  {
    if (step_dir==0)
    {
      pos_curr++;
    }else if(step_dir==1)
    {
      pos_curr--;
    }
    step_cnt++;
  }

  seq_cnt=(seq_cnt+1)%8;
  
}

// this fuction maps a double from one range to another
double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
