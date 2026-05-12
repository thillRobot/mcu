#define USING_TIMER_TC3 true 

#include <LiquidCrystal_I2C.h>
#include "SAMDTimerInterrupt.h"

//#include Wire.h



//initialize the liquid crystal library

//the first parameter is  the I2C address

//the second parameter is how many rows are on your screen

//the  third parameter is how many columns are on your screen
LiquidCrystal_I2C lcd(0x27,  16, 2);

const int buttonPin = 1; // Pin 1 is used here
volatile bool state = LOW;

int timer_cnt=0;
int isr_cnt=0;
float isr_freq=0;

// Select a hardware timer (TC3, TC4, TC5, TCC, TCC1, or TCC2)
SAMDTimer ITimer(TIMER_TC3); 

void TimerHandler() {
  static bool ledState = LOW;
  //digitalWrite(LED_BUILTIN, ledState);
  //ledState = !ledState;
  timer_cnt++;
  isr_freq=isr_cnt;
  isr_cnt=0;
  //lcd.setCursor(0,1);
}

volatile bool isProcessing = false;
// Interrupt Service Routine (ISR)
void blinkISR() {
  //state = !state; // Toggle state
  if (isProcessing) return; // Ignore if already processing
  isProcessing = true;

  for (int i=0;i<500;i++); // tiny debouncing delay 

  uint32_t portA = REG_PORT_IN0;
  bool pin1State = (portA >> 1) & 1; // True if HIGH, False if LOW    
  while(pin1State){
    pin1State = (portA >> 1) & 1; // True if HIGH, False if LOW    
  }; 

  isr_cnt++;
  isProcessing = false; // Allow new interrupts
  //digitalWrite(LED_BUILTIN, state);
}

void setup() {
  
  //initialize lcd screen
  lcd.init();
  
  // turn on the backlight
  lcd.backlight();

  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize timer with 500ms interval
 if (ITimer.attachInterruptInterval(500 * 1000, TimerHandler)) {
   Serial.println("Starting ITimer OK");
 } else {
   Serial.println("Can't set ITimer. Select another timer.");
 }

 //pinMode(buttonPin, INPUT_PULLUP);
 pinMode(buttonPin, INPUT);
  // Attach interrupt: call blinkISR when button goes from high to low
 attachInterrupt(digitalPinToInterrupt(buttonPin), blinkISR, RISING);   

 //pinMode(1, INPUT);    

}

unsigned long lastRun;

void loop() {
  
//wait  for a second
  
  if (millis() - lastRun >= 1000) {
  lastRun = millis();
  // do something
  //}

  lcd.clear(); 
// tell the screen to write on the top row
  lcd.setCursor(0,0);
  
// tell the screen to write “hello, from” on the top  row
//  lcd.print("Hello, From");
  
// tell the screen to write on the bottom  row
  lcd.setCursor(0,1);
  lcd.print(isr_cnt);
  
  lcd.setCursor(0,2);
  lcd.print(isr_freq);
// tell the screen to write “Arduino_uno_guy”  on the bottom row
  
// you can change whats in the quotes to be what you want  it to be!
// lcd.print("Arduino_uno_guy");
  }
}
