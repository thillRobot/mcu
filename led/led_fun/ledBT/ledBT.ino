// THis is Trinket 5v talking to HC06 Blue module 
// Sample application is RGB addressable LEDs
// for http://www.genotronex.com/

// you will need arduino 1.0.1 or higher to run this sketch

#include <SoftwareSerial.h>// import the serial library
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>

#define DPIN 0
#define RXPIN 1
#define TXPIN 2
#define LEDPIN 3

#define ANOFF 1
#define ANON 0

#ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.
 #include <avr/power.h>
#endif
 
SoftwareSerial Genotronex(RXPIN,TXPIN); // RX, TX
int ledpin=13; // led on D13 will show blink on / off
int BluetoothData; // the data given from Computer
unsigned int colorRGB[3];

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(3, DPIN);

// Define Global Vars Here
uint8_t  mode   = 0, // Current animation effect
         offset = 0; // Position of spinny eyes
uint32_t color  = 0x000000; // Start red BRG ( must be the strip)
uint32_t prevTime;
  
//define 'cool' colormap here RGB(0-10000)
uint8_t primes[3][3]={{255,0,0},
                      {0,255,0},
                      {0,0,255}};

uint8_t cool[64][3]= {{  0 ,  0 ,143 },
                      {  0 ,  0 ,159 },
                      {  0 ,  0 ,175 },
                      {  0 ,  0 ,191 },
                      {  0 ,  0 ,207 },
                      {  0 ,  0 ,223 },
                      {  0 ,  0 ,239 },
                      {  0 ,  0 ,255 },
                      {  0 , 16 ,255 },
                      {  0 , 32 ,255 },
                      {  0 , 48 ,255 },
                      {  0 , 64 ,255 },
                      {  0 , 80 ,255 },
                      {  0 , 96 ,255 },
                      {  0 ,112 ,255 },
                      {  0 ,128 ,255 },
                      {  0 ,143 ,255 },
                      {  0 ,159 ,255 },
                      {  0 ,175 ,255 },
                      {  0 ,191 ,255 },
                      {  0 ,207 ,255 },
                      {  0 ,223 ,255 },
                      {  0 ,239 ,255 },
                      {  0 ,255 ,255 },
                      { 16 ,255 ,239 },
                      { 32 ,255 ,223 },
                      { 48 ,255 ,207 },
                      { 64 ,255 ,191 },
                      { 80 ,255 ,175 },
                      { 96 ,255 ,159 },
                      {112 ,255 ,143 },
                      {128 ,255 ,128 },
                      {143 ,255 ,112 },
                      {159 ,255 , 96 },
                      {175 ,255 , 80 },
                      {191 ,255 , 64 },
                      {207 ,255 , 48 },
                      {223 ,255 , 32 },
                      {239 ,255 , 16 },
                      {255 ,255 ,  0 },
                      {255 ,239 ,  0 },
                      {255 ,223 ,  0 },
                      {255 ,207 ,  0 },
                      {255 ,191 ,  0 },
                      {255 ,175 ,  0 },
                      {255 ,159 ,  0 },
                      {255 ,143 ,  0 },
                      {255 ,128 ,  0 },
                      {255 ,112 ,  0 },
                      {255 , 96 ,  0 },
                      {255 , 80 ,  0 },
                      {255 , 64 ,  0 },
                      {255 , 48 ,  0 },
                      {255 , 32 ,  0 },
                      {255 , 16 ,  0 },
                      {255 ,  0 ,  0 },
                      {239 ,  0 ,  0 },
                      {223 ,  0 ,  0 },
                      {207 ,  0 ,  0 },
                      {191 ,  0 ,  0 },
                      {175 ,  0 ,  0 },
                      {159 ,  0 ,  0 },
                      {143 ,  0 ,  0 },
                      {128 ,  0 ,  0 }};

void setup() {
  
  #ifdef __AVR_ATtiny85__ // Trinket, Gemma, etc.
  if(F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  pixels.begin();
  pixels.show();
  pixels.setBrightness(85); // 1/3 brightness
  prevTime = millis();

  // put your setup code here, to run once:
  Genotronex.begin(9600);
  Genotronex.println("Bluetooth On please press 1 or 0 blink LED ..");
  pinMode(LEDPIN,OUTPUT);

  digitalWrite(ledpin,0);
  Genotronex.println("LED  On D3 ON ! ");  
}

void loop() {
    // put your main code here, to run repeatedly:
  
   if (Genotronex.available()){
        BluetoothData=Genotronex.read();
        if(BluetoothData=='1'){   // if number 1 pressed ....
          digitalWrite(LEDPIN,0);
          Genotronex.println("LED  On D3 ON ! ");
        }
        if (BluetoothData=='0'){// if number 0 pressed ....
          digitalWrite(LEDPIN,1);
          Genotronex.println("LED  On D3 Off ! ");
        }
     
     
      
        if ((BluetoothData=='R')){// if number 'blue' is pressed ....

          Genotronex.println("LED STRIP RED ! ");
          pixels.setPixelColor(0,primes[1][2],primes[1][1],primes[1][0]);
          pixels.show();
          delay(1000);
        }
        if (BluetoothData=='G'){// if number 'blue' is pressed ....

          Genotronex.println("LED STRIP GREEN ");
          pixels.setPixelColor(0,primes[0][2],primes[0][1],primes[0][0]);
          pixels.show();
          delay(1000);
        }
        if (BluetoothData=='B'){// if number 'blue' is pressed ....

          Genotronex.println("LED STRIP BLUE ! ");
          pixels.setPixelColor(0,primes[2][2],primes[2][1],primes[2][0]);
          pixels.show();
          delay(1000);
        }
    }
    
  delay(100);// delay for next data ... :(
}

  /*
  int pos = 0;  
  double rads;
  double degs;
  
  for (int j=0;j<=63;j++){  
    colorRGB[0]=cool[j][0];
    colorRGB[1]=cool[j][1];
    colorRGB[2]=cool[j][2];
    pixels.setPixelColor(0, colorRGB[2], colorRGB[0], colorRGB[1]);
    pixels.show();
    delay(100);
  }  
  
   for (int j=63;j>=0;j--){
    colorRGB[0]=cool[j][0];
    colorRGB[1]=cool[j][1];
    colorRGB[2]=cool[j][2];
    pixels.setPixelColor(0, colorRGB[2], colorRGB[0], colorRGB[1]);
    
    pixels.show();
    delay(100);
  }
  */



 
                      

