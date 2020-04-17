//
// Microstep Driver Tb6600 Interface
// Tristan Hill - March, 04, 2019
// 
// This uses the a while loop and the TOV1 flag to handle the step counting

/**************** Wiring Diagram******************* 

        MEGA2560     |     |  MICROSTEP DRIVER
                     |     |
                   5V|<--->|NC
                  GND|<--->|ENA(-) 
                  GND|<--->|DIR(-)
                  GND|<--->|PUL(-)  
          Pin22 (PWM)|<--->|ENA(+)
          Pin23 (PA0)|<--->|DIR(+)
          Pin23 (PA1)|<--->|PUL(+)
                     |     |
                           |
    STEPPER MOTOR    |     |    
                     |     |
                  RED|<--->|A(+)
                 BLUE|<--->|A(-)
                BLACK|<--->|B(+)
               GREEEN|<--->|B(-)
                     |     |
                           |  
       POWER SUPPLY  |     |
                     |     |       
              V+(12v)|<--->|VCC
              V-(0v) |<--->|GND
                     |     |

*/
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

bool motor_enable=0;  // turn off the motor to begin
bool motor_dir=1;     // set the direction
bool motor_status=1;  // 0 - 'not currently stepping' // 1 - 'currently stepping'  

int motor_pos=0;
                      

// function for stepping by number of steps (blocking)
void set_steps(int steps)
{
  while (!(TIFR1 & 0b00000010));  // wait for flag to start
  PORTA &= 0b11111101|(motor_dir<<1);             // set the direction  
  PORTA &= 0b11111110|~motor_enable;              // set enable low('on')
  //delay(1.0);
  for (int k = 0; k < steps+1; k++) {
    while (!(TIFR1 & 0b00000010));  // wait for flag 
    TIFR1 |= 0b00000010;            // clear the flag
    motor_pos=(motor_pos+motor_dir)%200;
  }
  
  PORTA |= 0b00000001|~motor_enable;   // set enable high('off')
}

/*
// function for stepping to position (blocking)
void set_steps(int steps)
{
  PORTA &= 0b11111101|(motor_dir<<1);             // set the direction  
  PORTA &= 0b11111110|~motor_enable;              // set enable low('on')
  
  for (int k = 0; k < steps; k++) {
    while (!(TIFR1 & 0b00000010));  // wait for flag 
    TIFR1 |= 0b00000010;            // clear the flag
  }
  
  PORTA |= 0b00000001|~motor_enable;   // set enable high('off')
}
*/
// setup ROS node
ros::NodeHandle nh;

// publisher object for motor status
std_msgs::Bool status_msg;
ros::Publisher status_pub("motor_status", &status_msg);

// publisher object for motor position
std_msgs::Int32 pos_msg;
ros::Publisher pos_pub("motor_position", &pos_msg);

// Callback function for 'motor enable'
void enable_cb( const std_msgs::Bool& msg){
  motor_enable=msg.data;   // turn on the motor
}
// Subscriber object for 'motor enable'
ros::Subscriber<std_msgs::Bool> sub1("motor_enable", &enable_cb );

// Callback function for 'motor direction'
void dir_cb( const std_msgs::Bool& msg){
  motor_dir=msg.data;   // set the direction
}
// Subscriber object for 'motor direction'
ros::Subscriber<std_msgs::Bool> sub2("motor_dir", &dir_cb );

/*
// Callback function for 'desired position'
void dir_cb( const std_msgs::Bool& msg){
  motor_dir=msg.data;   // set the desired pos
}
// Subscriber object for 'motor direction'
ros::Subscriber<std_msgs::Bool> sub2("motor_dir", &dir_cb );
*/

// Callback for 'command steps'
void cmd_steps_cb( const std_msgs::Int32& msg){
  motor_status=1;
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();
  
  set_steps(msg.data);  // command the steps

  pos_msg.data = motor_pos;
  pos_pub.publish( &pos_msg );
  
  motor_status=0;
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  nh.spinOnce();
}
// Subscriber object for 'command steps'
ros::Subscriber<std_msgs::Int32> sub3("cmd_steps", &cmd_steps_cb );

void setup() {
  // put your setup code here, to run once:

  // PWM OUTPUT
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode

  // 10 bit fast pwm mode, normal port operation
  TCCR1A |= (1 << COM1A1) | (1 << WGM11) | (1 << WGM10);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  // I cant figure out why i hvae to turn OFF the bits in CS0:2 (TWH)

  // prescale 1 (none)
  //TCCR1B |= (1<<WGM12)  | (1<<CS10);                //CS12:0=001 -> 15.63 kHz, T=63.97 us
  //TCCR1B &= ~(1<<CS11) & ~(1<<CS12);

  // prescale 8
  //TCCR1B |= (1 << WGM12)  | (1 << CS11) ;           //CS12:0=010 -> 1.95 kHz , T=512.8 us
  //TCCR1B &= ~(1 << CS10) & ~(1 << CS12);

  // prescale 64
  //TCCR1B |= (1<<WGM12)  | (1<<CS11) | (1<<CS10);    //CS12:0=011 -> 244.1 Hz, T=4098 us
  //TCCR1B &= ~(1<<CS12);                             //              122.2 Hz  

  // prescale 256
  TCCR1B |= (1<<WGM12)  | (1<<CS12);                //CS12:0=100 -> 61.0 Hz ,T=16393 us
  TCCR1B &= ~(1<<CS10) & ~(1<<CS11);

  // prescale 10241
  //TCCR1B |= (1<<WGM12)  | (1<<CS12) | (1<<CS10);      //CS12:0=101 -> 15.3 Hz ,T=65359 us
  //TCCR1B &= ~(1<<CS11);

  // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR1A = 500 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  // 500/16393*1023 -> 500 us pulse at 61. Hz

  DDRB |=  0b11111111; // PA0,PA1 set as output

  DDRA  =  0b11111111; // Port A as output
  //PORTA &= 0b11111100|(motor_dir<<1); // set enable low ('on') and direction low('cw')
  PORTA |= 0b00000001;  // set enable high('off')
  
  // setup serial monitior
  // Serial.begin(9600);          //  setup serial
  // while (! Serial);
  // Serial.println("Microstep Driver Setup!");
  
  // setup ROS node
  nh.initNode();
  
  // setup subscribers
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  delay(.01);
  
  nh.spinOnce();
  delay(.01);
  
  // setup publishers
  nh.advertise(status_pub);
  nh.advertise(pos_pub);
  
}

void loop() {

  pos_msg.data = motor_pos;
  pos_pub.publish( &pos_msg );
 
  status_msg.data = motor_status;
  status_pub.publish( &status_msg );
  
  nh.spinOnce();
  delay(.01);
  
}
