//
// Mechatronics ME4370 - Tennessee Technological University
// Tristan Hill - April 29, 2016 
// Example of the 8 bit Timer/Counter 0 !!! 


// this cannot be done as far as I can tell without modifying wiring.c 
// super lame , i would like to try but i dont have time this morning!!! 
// the built in functions like delay() and millis() use TIMER0 so and i cant refine the ISR

//define global vars here
volatile uint16_t ofcnt= 0; 

ISR(TIMER0_OVF_vect) {

    ofcnt++;
   // time_sec=time_sec+.0041/2.0; // need to figure this number out! 1/16e6*65536/2
}

void setup() {
  

   // setup serial monitior
  //Serial.begin(250000);     
  //while (! Serial);
  //delay(500);
  //Serial.println("8 bit timer example - TTU - Sping 2016");


  // put your setup code here, to run once:
  // 8 bit Timer 0 for general scheduling (delay)
  TCCR0A=0b00000000;  // all bits off - Normal Mode (non-PWM)
                      // Normal port operation, OC0A disconnected
                      // Normal port operation, OC0B disconnected
  TCCR0B=0b00000001;  // No Prescaling (PRSCL=1)
  TIMSK0=0b00000001;  // Overflow interrupt enable


}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(ofcnt);
}


