//
// LMD18200 H-BRIDGE Interface for driving a Bi-Polar Stepper Motor
// Tristan Hill - March, 06, 2019
// 
// This is very primative and needs work... but it works!

/**************** Wiring Diagram******************* 

  MEGA2560                 LMD18200 (1)
  ________                 ___________                     

                          pin1(bootstrap input1)<---> 10nF cap to pin2
                          pin2(motor output 1)  <---> stepper yellow (mercury motor SM42BYG011-25)
        PA0 (pin22) <---> pin3(direction input) 
        PA1 (pin23) <---> pin4(brake input)
        PA2 (pin24) <---> pin5(PWM input)  
                          pin6(VS power) <--------> 12v
                GND <---> pin7(GND))<-------------> GND 
                          pin8(current sense)
                          pin9(thermal flag)
                          pin10(motor output 2) <---> stepper blue
                          pin11(bootstrap input 2) <---> 10nF cap to pin10

                           LMD18200 (2)
                           ___________ 
                            
                          pin1(bootstrap input1)<---> 10nF cap to pin2
                          pin2(motor output 1)  <---> stepper red
        PA3 (pin25) <---> pin3(direction input) 
        PA4 (pin26) <---> pin4(brake input)
        PA5 (pin27) <---> pin5(PWM input)  
                          pin6(VS power) <--------> 12v
                GND <---> pin7(GND))<-------------> GND 
                          pin8(current sense)
                          pin9(thermal flag)
                          pin10(motor output 2) <---> stepper green
                          pin11(bootstrap input 2) <---> 10nF cap to pin10


*/
// this looks much different using this lmd18200 chip
// setting the outpout states is counterintuitive. just use the given truth table
// PORTA = 0b00000101;  // Source 1, Sink 2     pin2:high pin10:low     
// PORTA = 0b00000100;  // Source 1, Source 2   pin2:low  pin10:high     
// PORTA = 0b00000001;  // Sink 1, Source 2     pin2:high pin10:high    
// PORTA = 0b00000110;  // Sink1, Sink2         pin2:low  pin10:low
// PORTA = 0b00000001;  // None, None           pin2:-    pin10:-      
  
void cmd_fullstep(int steps, bool dir);
void cmd_halfstep(int steps, bool dir);

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
               
void setup() {
  // put your setup code here, to run once:
  
  // setup serial monitior
  // Serial.begin(9600);          //  setup serial
  // while (! Serial);
  // Serial.println("LMD18200 Driver Setup!");
  
  DDRA=0b11111111;
}

void loop() {

  cmd_halfstep(50,1,5);
  delay(100);
  //cmd_halfstep(20,0,5);
  delay(100);
  
}

void cmd_fullstep(int steps,bool dir, int dt)
{

  if (dir==1)
  {  
    for(int k=0;k<steps;k++)
    {
      for(int j=0;j<4;j++)
      {
        PORTA = fullstep[j]; // low high, low high
        delay(dt);
      }
    }
  }else
  {  
    for(int k=0;k<steps;k++)
    {
      for(int j=3;j>=0;j--)
      {
        PORTA = fullstep[j]; // low high, low high
        delay(dt);
      }
    }
  }
}

void cmd_halfstep(int steps,bool dir,int dt)
{
  if (dir==1)
  {  
    for(int k=0;k<steps;k++)
    {
      for(int j=0;j<8;j++)
      {
        PORTA = halfstep[j]; // low high, low high
        delay(dt);
      }
    }
  }else
  {  
    for(int k=0;k<steps;k++)
    {
      for(int j=7;j>=0;j--)
      {
        PORTA = halfstep[j]; // low high, low high
        delay(dt);
      }
    }
  }
}
