//
// Tennessee Technological University
// Tristan Hill - Jan 12, 2016 , updated May 05, 2020
//              - again Aug 12, 2020 

// This script is for the Arduino Mega2560 in the 'Door Mechanism' control box
// the arduino interfaces witht the Roboteq SDC2160 Motor Controller and receives 
// the 'optical feedback' signal from the firgelli linear actuator  

// For testing there is pot on the Mega
// An analog voltage from a potentiometer is converted to a 10 bit value with ADC.
// The value is copied directly to a PWM duty value and the PWM
// signal is output directly to the analog input (0-5v) on the SDC2160 motor controller.
// This is very crude but it works suprisingly well.
//
// Now, the MCU is counting pulses from the actuator but there is only one pulse signal 
// so there is no direction  ?!?!? im stuck !?!?!
// user def funs prototypes here

//double map_double(double x, double in_min, double in_max, double out_min, double out_max);


// global from ADC
int adc_val;

int mot_val;

int ref_counts;

int err_counts;

int max_counts=2000;
int min_counts=-2000;


// interrupt counters
volatile int16_t counts=0;


// define the ISR functions
ISR(INT4_vect)
{
  if (mot_val>350){
    counts++;
  }else{
    counts--;
  }
  
  //PORTB |= 0b00000001; // turn on LED
  TIFR4 |= (1<<ICF4); //clear the flag (not sure this is working)  
}



void setup() {
  // put your setup code here, to run once:
    
  // setup serial monitior
  Serial.begin(9600);          //  setup serial
  while (!Serial);
  Serial.println("Door Mechanism - ME4370!");

  // PWM OUTPUT 
  // Set up timer to toggle OC1A (PB5, pin11) on match of OCR1A in Fast PWM mode
  // also add  OC1B (PB6, pin12) on match of OCR1B in Fast PWM mode
  
  // 10 bit fast pwm mode, normal port operation
  TCCR1A |= (1<<COM1A1) | (1<<WGM11) | (1<<WGM10) | (1<<COM1B1);
  // set up timer with prescaler and Fast 10 bit PWM mode (mode 7, bits WGM13:0)
  
  // prescale 1 (none)
  TCCR1B |= (1<<WGM12)  | (1<<CS10);                //CS12:0=001 -> 15.63 kHz          
  TCCR1B &= ~(1<<CS11) & ~(1<<CS12);
  
  // Set compare value to 1Hz at 16MHz AVR clock ???
  // whenever a match occurs OC1A toggles
  OCR1A = 555 ; // this values controls the duty cycle, duty(%)=OCR1A/255*100
  OCR1B = 400 ; // 0-300 is reverse       // I am running a PWM -> Analog Input directly
                // 300-400 is the center  // the deadband is adjustable in Roborun (roboteq)
                // 400-700 is forward    // these are very approximate  
                // past that is dead for some reason 
  
  //DDRB |= B00100000; // PB5 (pin 11 on mega) as output 
  DDRB |= (1<<PB5) | (1<<PB6); //data direction register

  // setup analog to digital converter  
  ADMUX |= B01100000;
  // Set ADC reference to AVREF (bit7-6,REFS1-0)
  // Left adjust ADC result to allow easy 8 bit reading (bit5,ADLAR)
  // Use AC0 (bit4-0,MUX4-0) 
  
  ADCSRA |= B10000111;
  // Enable ADC (bit7,ADEN)
  // Start A2D Conversions (bit6,ADSC)
  // Dont Enable auto trigger (bit5,ADATE)
  // Clear Interrupt Flag (bit4,ADIF)
  // Enable Interrupts (bit3,ADIE)
  // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz (bit 0-2,ADPS2-0)

  //setup external interrupts, INT4(PE4,pin2)
  cli(); //turn interrupts off during setup
  DDRE &= 0b11101111; // PE4 as input
  EICRB |= 0b00000011;  // 'rising edge' trigger on INT4 (table 15-3)
  EIMSK |= 0b00010000; //external interrupt request enable on INT4 and INT5
  sei(); //turn interrupts on for use
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  delay(500);

  ADCSRA |= (1<<ADSC); //start conversions
  while (!(ADCSRA&B00010000)); // wait for conversion flag(ADIF)
  adc_val=ADCL>>6|ADCH<<2; // IMPORTANT !!! access ADCL before ADCH !!! 
  
//  Serial.println(val);
  //OCR1B = val ; // 0-300
  ref_counts=map(adc_val,0,1024,0,5000);
  err_counts=(ref_counts-counts);
  
  mot_val=map(err_counts*2.0,-5000,5000,0,700);
  OCR1B=mot_val;

  Serial.println("Reference Counts:");
  Serial.println(ref_counts);
  Serial.println("Measured Counts:");
  Serial.println(counts);
  Serial.println("Error Counts:");
  Serial.println(err_counts);
  Serial.println("Motor Command:");
  Serial.println(mot_val);
}

/*
// this fuction maps a double from one range to another
double map_double(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/
