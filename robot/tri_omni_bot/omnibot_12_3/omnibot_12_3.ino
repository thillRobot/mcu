// TTU Swedish Bot, Built
//
// Interfacing the 8bit NES controller with Arduino Duemilanove
// And using it to control the 3 sweedish wheel omnibot

// Also has beginnings of ANDRIODD interface
/* INITIALIZATION */

#include <Servo.h> 
#include <SoftwareSerial.h>// import the serial library

#define UP     0b00001000   //lookup map for NES controller 
#define DOWN   0b00000100   // can be replaced, notice the shift
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
float cmds[3]={500,500,500}; // // robot velocities as array
char  flgs[3]={'!','@','#'};

int latch = 2; // set the latch pin - orange
int clock = 3; // set the clock pin - red
int datin = 4;// set the data in pin - yellow
byte controller_data = 0;
int ledpin = 13;
int buttons;
int btData; //characters from BT
int btFlag=1;

SoftwareSerial myBT(6, 7); // RX, TX

/* SETUP */
void setup() {
  
  myBT.begin(9600);
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
  
  set_velocity(cmds[0],cmds[1],cmds[2]);
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
  
  //set the robot velocity
  
  Serial.println("Setting Robot Velocity"); 
  set_velocity(cmds[0],cmds[1],cmds[2]);
  
  if (btFlag)
  {
    // Bluetooth Communication Mode
    Serial.println("entering bluetooth mode");
    int i=0;
    while((i<3)&&btFlag)
    {
      int waiting=1;
      int reading=1;
      while(waiting&&btFlag)
      {  
        while(!myBT.available());
        btData=myBT.read();
        if (btData==flgs[i])
        {
          waiting=0;
        }else if (btData=='*')
        {
          Serial.println("exiting bluetooth mode");
          btFlag=0;
        }
      }
      //Serial.println("exiting complete1");
      if (btFlag)
      {
        float valu=0;
        for(int j=0;j<3;j++)
        {
          while(!myBT.available());
          btData=myBT.read();
          //Serial.println(btData, DEC);
          valu=valu+(btData-48)*pow(10,2-j);
          //Serial.println(valu, DEC);
        }
        //Serial.println(valu, DEC);
        cmds[i]=valu;
        //Serial.println(cmds[i], 2);
        
        //Serial.println("Command Received");
       
      }
      i++;
    }
    
  }
} 

// this fuction maps a double from one range to another
float map_double(float x, double in_min, double in_max, double out_min, double out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// this function takes in three robot velocites, vel_x, vel_y, omega
// and maps them to proper values and commands the servos to drive the robot
void set_velocity(float vel_x, float vel_y, float omega){
  //omega=-omega;
  vel_x=vel_x-500;
  vel_y=vel_y-500;
  omega=omega-500;
  Serial.println(vel_x,DEC);
  Serial.println(vel_y,DEC);
  Serial.println(omega,DEC);
  delay(100);
  float w1=(-vel_y+L*omega)*(1/r);                            // kinematic equations for omnibot
  float w2=(-0.5*sqrt(3)*vel_x + 0.5*vel_y + L*omega)*(1/r);  // compute wheel vels from robot vels
  float w3=(0.5*sqrt(3)*vel_x + 0.5*vel_y + L*omega)*(1/r);
  Serial.println(w1,DEC);
  Serial.println(w2,DEC);
  Serial.println(w3,DEC);
  Serial.println(' ');
  // scale it to use it with the servo (value between 0 and 180) 
  // set the servo velocity according to the scaled value
  
  
  float mn1=-100;
  float mn2=-100;//magic numbers - derive later 
  float mn3=-100;
  float mx1=100;
  float mx2=100;
  float mx3=100;
  int w1_deg=map_double(w1, mn1, mx1, 0, 179);
  int w2_deg=map_double(w2, mn1, mx1, 0, 179);
  int w3_deg=map_double(w3, mn1, mx1, 0, 179);
  Serial.println(w1_deg,DEC);
  Serial.println(w2_deg,DEC);
  Serial.println(w3_deg,DEC);
  servo1.write(w1_deg);                   
  servo2.write(w2_deg);                   
  servo3.write(w3_deg);                  
  
}
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
