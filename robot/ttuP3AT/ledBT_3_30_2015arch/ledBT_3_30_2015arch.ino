// TTU P3AT
// 
// Code for onboard Arduino Mega 2560
// 
// Also has beginnings of ANDRIODD interface
/* INITIALIZATION */

//#include <Servo.h> 
#include <SoftwareSerial.h>// import the serial library
#include <Adafruit_NeoPixel.h>

//#ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.
 //#include <avr/power.h>
//#endif
 
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(3, 6);
// Define Global Vars Here

float cmds[3]={0,0,0}; // // robot velocities as array
char  flgs[4]={'!','@','#','$'};
int sel=0;

int btData; //characters from BT
SoftwareSerial myBT(10, 11); // RX, TX, note: only subset of IO works for software serial on mega


/* SETUP */
void setup() {
 // #ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.\
//  if(F_CPU == 16000000) clock_prescale_set(clock_div_1);
 // #endif
  
  pixels.begin();
  pixels.show();
  
  // put your setup code here, to run once:
  
  myBT.begin(9600);
  
}



/* THE LED, SERVO, AND SERIAL MONITOR PROGRAM */
void loop() {
  
      // now send turn on LEDs 
    pixels.setPixelColor(sel,cmds[2],cmds[0],cmds[1]);
    pixels.show();
    delay(100); 
    
    int i=0;
    while(i<4)
    {
      int waiting=1;
      while(waiting)
      {  
        while(!myBT.available());
        btData=myBT.read();
        if (btData==flgs[i])
        {
          waiting=0;
        }
      }
      
        if (i<3){
          float valu=0;
          for(int j=0;j<3;j++)
          {
            while(!myBT.available());
            btData=myBT.read();
            if (j==0){
              valu=valu+(btData-48)*10*10;
            }
            if (j==1){
              valu=valu+(btData-48)*10;
            }
            if (j==2){
              valu=valu+(btData-48);
            }
          }
          cmds[i]=valu-500;
        }
        else{
            while(!myBT.available());
            btData=myBT.read();
            sel=btData-48;
        }
   
      i++;
    }
}

