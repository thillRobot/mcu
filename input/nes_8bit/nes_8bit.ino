// Interfacing the 8bit NES controller with Arduino Duemilanove
// And using it to control the 3 sweedish wheel omnibot
/* INITIALIZATION */

#include <Servo.h> 

#define UP     0b00001000
#define DOWN   0b00000100
#define LEFT   0b00000010
#define RIGHT  0b00000001 
#define SELECT 0b00100000
#define START  0b00010000
#define A      0b10000000
#define B      0b01000000

// user def funs prototypes here
float map_double(double x, double in_min, double in_max, double out_min, double out_max);
void set_velocity(float vel_x, float vel_y, float omega);

Servo servo1,servo2,servo3;  // create 3 servo objects to control the omnibot 
 
float r=2.2; //wheel radius cm
float L=14;  //distance from 
float vx=0,vy=0,w=0; // robot velocities

int latch = 2; // set the latch pin - orange
int clock = 3; // set the clock pin - red
int datin = 4;// set the data in pin - yellow
byte controller_data = 0;
int ledpin = 13;
int buttons;

/* SETUP */
void setup() {
  Serial.begin(57600);
  pinMode(latch,OUTPUT);
  pinMode(clock,OUTPUT);
  pinMode(datin,INPUT);
  pinMode(ledpin, OUTPUT);
  
  digitalWrite(latch,HIGH);
  digitalWrite(clock,HIGH);
  
  servo1.attach(9);  // attaches the servo on pin 9 to the servo object
  servo2.attach(10);  // attaches the servo on pin 10 to the servo object 
  servo3.attach(11);  // attaches the servo on pin 11 to the servo object 
}

/* THIS READS DATA FROM THE controller */
void controllerRead() {
  controller_data = 0;
  digitalWrite(latch,LOW);
  digitalWrite(clock,LOW);
  
  digitalWrite(latch,HIGH);
  delayMicroseconds(2);
  digitalWrite(latch,LOW);
  
  controller_data = digitalRead(datin);
  
  for (int i = 1; i <= 7; i ++) {
    digitalWrite(clock,HIGH);
    delayMicroseconds(2);
    controller_data = controller_data << 1;
    controller_data = controller_data + digitalRead(datin) ;
    delayMicroseconds(4);
    digitalWrite(clock,LOW);
  }
}

/* THE LED, SERVO, AND SERIAL MONITOR PROGRAM */
void loop() {
  controllerRead();
  
  buttons=(~controller_data)&0xFF;
  
  Serial.println(buttons, BIN);
  
  // controller Scheme A
  /*
  if (controller_data==UP){
    vx=vx+.05;   
    }
    
  if (controller_data==DOWN){
    vx=vx-.05;
   }
  
  if (controller_data==LEFT){
    vy=vy+.05;   
    }
    
  if (controller_data==RIGHT){
    vy=vy-.05;
   }
  
  if (controller_data==(LEFT&B)){
    w=w-.002;
   }
  
  if (controller_data==(RIGHT&B)){
    w=w+.002;
   }
     
  if (controller_data==(START&SELECT)){
    vx=0;vy=0;w=0;
   }
  */
  
  // controller Scheme B
  
  vy=0;vx=0;w=0; // all velocities default at 0
  
  if (buttons&UP){
    vx=.5;   
  } else if (buttons&DOWN){
    vx=-.5;
  } else if (buttons&LEFT){
    vy=.5;w=.005; // the omega here is a correction   
  } else if (buttons&RIGHT){
    vy=-.5;w=-.0065; // the omega here is a correction
  } 
  
  if ((buttons&UP)&&(buttons&LEFT)){
    vx=.5;vy=.5;w=-.0065;
  } else if ((buttons&UP)&&(buttons&RIGHT)){
    vx=.5;vy=-.5;w=.005;
  }else if ((buttons&DOWN)&&(buttons&LEFT)){
    vx=-.5;vy=.5;w=-.008;
  } else if ((buttons&DOWN)&&(buttons&RIGHT)){
    vx=-.5;vy=-.5;w=.006;
  }
  
  if  (buttons&A)
  {
    w=-.1;
  }else if (buttons&B)
  {
    w=.1;
  }
  
  if ((buttons&A)&&(buttons&UP))
  {
    vx=.5;
    w=-.02;
  }
  if ((buttons&B)&&(buttons&UP))
  {
    vx=.5;
    w=.02;
  }
  
  
  
  // now set the robot velocity
  set_velocity(vx,vy,w);
    
  delay(100); // delay for loop()    
} 

// this fuction maps a double from one range to another
float map_double(double x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// this function takes in three robot velocites, vel_x, vel_y, omega
// and maps them to proper values and commands the servos to drive the robot
void set_velocity(float vel_x, float vel_y, float omega){
  omega=-omega;
  float w1=(-vel_y+L*omega)*(1/r);                            // kinematic equations for omnibot
  float w2=(-0.5*sqrt(3)*vel_x + 0.5*vel_y + L*omega)*(1/r);  // compute wheel vels from robot vels
  float w3=(0.5*sqrt(3)*vel_x + 0.5*vel_y + L*omega)*(1/r);
 
  // scale it to use it with the servo (value between 0 and 180) 
  // set the servo velocity according to the scaled value
  servo1.write(map_double(-w1, -1, 1, 0, 179)+6);                   
  servo2.write(map_double(-w2, -1, 1, 0, 179)+6);                   
  servo3.write(map_double(-w3, -1, 1, 0, 179)+6);                  
}

