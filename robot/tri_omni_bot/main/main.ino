// Interfacing the 8bit NES controller with Arduino Duemilanove
// And using it to control the 3 sweedish wheel omnibot
/* INITIALIZATION */

#include <Servo.h> 

// user def funs prototypes here
float map_double(double x, double in_min, double in_max, double out_min, double out_max);

Servo servo1,servo2,servo3;  // create 3 vservo objects to control the omnibot 
 
float r=2.2; //wheel radius cm
float L=14;  //distance from 
float vx=0,vy=0,w=0; // robot velocities
float w1,w2,w3; // wheel velocities
float sp1,sp2,sp3; // inputs to servo function

int latch = 2; // set the latch pin
int clock = 3; // set the clock pin
int datin = 4;// set the data in pin
byte controller_data = 0;
int ledpin = 13;

/* SETUP */
void setup() {
  Serial.begin(57600);
  pinMode(latch,OUTPUT);
  pinMode(clock,OUTPUT);
  pinMode(datin,INPUT);
  pinMode(ledpin, OUTPUT);
  
  digitalWrite(latch,HIGH);
  digitalWrite(clock,HIGH);
  
  servo1.attach(10);  // attaches the servo on pin 10 to the servo object 
  servo2.attach(9);  // attaches the servo on pin 9 to the servo object
  servo3.attach(11);  // attaches the servo on pin 11 to the servo object 
}

/* THIS READS DATA FROM THE CONTROLLER */
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
  Serial.println(controller_data, BIN);
  
  //lookups for buttons  
  int UP = 0b11110111;
  int DOWN = 0b11111011;
  int LEFT = 0b11111101;
  int RIGHT = 0b11111110;
  int SELECT=0b11011111;
  int START=0b11101111;
  int A= 0b01111111;
  int B= 0b10111111;
  
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
    vx=0;
    vy=0;
    w=0;
   }
    
  w1=(-vy+L*w)*(1/r);                         //kinematic equations
  w2=(-0.5*sqrt(3)*vx + 0.5*vy + L*w)*(1/r);  // compute wheel vels from robot vels
  w3=(0.5*sqrt(3)*vx + 0.5*vy + L*w)*(1/r);
   
  sp1 = map_double(-w1, -1, 1, 0, 179)+6;     // scale it to use it with the servo (value between 0 and 180) 
  sp2 = map_double(-w2, -1, 1, 0, 179)+6;      
  sp3 = map_double(-w3, -1, 1, 0, 179)+6;      
 
  servo1.write(sp1);                  // set the servo velocity according to the scaled value 
  servo2.write(sp2);                   
  servo3.write(sp3);                   
      
  delay(100); // delay for loop()    
} 

float map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
