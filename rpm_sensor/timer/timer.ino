#define USING_TIMER_TC3 true 

#include <LiquidCrystal_I2C.h>
#include "SAMDTimerInterrupt.h"

#define INTERRUPT_CORE_SAMD

#include "SAMD_PWM.h"

// Define a valid hardware PWM pin (e.g., Pin 2)
const int pinPWM = 2; 
float frequency = 100.0; // Target frequency in Hz
float dutyCycle = 1.0;  // 5% duty cycle (Common for 50Hz RC Servos)

SAMD_PWM* PWM_Instance;


//#include Wire.h



//initialize the liquid crystal library

//the first parameter is  the I2C address

//the second parameter is how many rows are on your screen

//the  third parameter is how many columns are on your screen
LiquidCrystal_I2C lcd(0x27,  20, 4);

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

  //for (int i=0;i<500;i++); // tiny debouncing delay 

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

// // Initialize timer with 500ms interval
if (ITimer.attachInterruptInterval(1000 * 1000, TimerHandler)) {
  Serial.println("Starting ITimer OK");
} else {
  Serial.println("Can't set ITimer. Select another timer.");
}

 //pinMode(buttonPin, INPUT_PULLUP);
 pinMode(buttonPin, INPUT);
  // Attach interrupt: call blinkISR when button goes from high to low
 attachInterrupt(digitalPinToInterrupt(buttonPin), blinkISR, RISING);   

//int pwmPin = 2; // Example PWM pin
 // pinMode(pwmPin, OUTPUT);

 // Create instance: Pin, Frequency, Duty Cycle
  PWM_Instance = new SAMD_PWM(pinPWM, frequency, dutyCycle);

  if (PWM_Instance) {
    PWM_Instance->setPWM();
  }

}                
unsigned long lastRun;

void loop() {
  
  // Set duty cycle to 50% (128/255)
  //analogWrite(2, 128); 
//wait  for a second
   
  if (millis() - lastRun >= 1000) {
  lastRun = millis();
  // do something
  //}
  
  lcd.clear(); 
// tell the screen to write on the top row
//  lcd.setCursor(0,0);
  
// tell the screen to write “hello, from” on the top  row
//  lcd.print("RPM Sensor");
  
// tell the screen to write on the bottom  row
  lcd.setCursor(0,0);
  lcd.print("Pulses per second:" );
  lcd.setCursor(0,1);
  lcd.print(isr_freq);
  lcd.setCursor(0,2);
  lcd.print("RPM: " );
  lcd.setCursor(0,3);
  float RPM=isr_freq*60;
  lcd.print(RPM);
  }
 
}
