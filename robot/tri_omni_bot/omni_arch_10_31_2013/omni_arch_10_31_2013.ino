// Controlling a servo position using a potentiometer (variable resistor) 
// by Michal Rinott <http://people.interaction-ivrea.it/m.rinott> 

#include <Servo.h> 

float map_double(double x, double in_min, double in_max, double out_min, double out_max);

 
Servo servo1,servo2,servo3;  // create 3 vservo objects to control the omnibot 
 
float r=2.2; //wheel radius cm
float L=14;  //distance from 
float vx,vy,w; // robot velocities
float w1,w2,w3; // wheel velocities
float sp1,sp2,sp3; // inputs to servo function

void setup() 
{ 
  servo1.attach(10);  // attaches the servo on pin 10 to the servo object 
  servo2.attach(9);  // attaches the servo on pin 9 to the servo object
  servo3.attach(11);  // attaches the servo on pin 11 to the servo object 
  
  Serial.begin(9600); // turn on serial 
} 
 
void loop() 
{ 
  vx=1;
  vy=0;
  w=0;

  w1=(-vy+L*w)*(1/r);  //kinematic equations
  w2=(-0.5*sqrt(3)*vx + 0.5*vy + L*w)*(1/r);
  w3=(0.5*sqrt(3)*vx + 0.5*vy + L*w)*(1/r);
  
  
  sp1 = map_double(-w1, -1, 1, 0, 179)+6;     // scale it to use it with the servo (value between 0 and 180) 
  sp2 = map_double(-w2, -1, 1, 0, 179)+6;     // scale it to use it with the servo (value between 0 and 180) 
  sp3 = map_double(-w3, -1, 1, 0, 179)+6;     // scale it to use it with the servo (value between 0 and 180) 
  
  
  servo1.write(sp1);                  // sets the servo position according to the scaled value 
  servo2.write(sp2);                  // sets the servo position according to the scaled value 
  servo3.write(sp3);                  // sets the servo position according to the scaled value 
  
  delay(15);                           // waits for the servo to get there 
}

float map_double(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

