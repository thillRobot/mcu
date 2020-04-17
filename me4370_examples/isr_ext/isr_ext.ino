/*  
 *  ME4370 - Tennessee Technological University   
 *  Tristan Hill - January 25, 2016 
 *  original code from 'skmishra91' at avrfreaks.net
 *  modified by TWH for arduino IDE
 *  
 *  Externally Trigger Interrupt Service Routines (timer capture)
 *  
 */
 
// interrupt counters
volatile uint8_t itp_4=0x00,itp_5=0x00;

// define the ISR functions
ISR(INT4_vect)
{
  itp_4++; 
  PORTB |= 0b00000001; // turn on LED
  TIFR4 |= (1<<ICF4); //clear the flag (not sure this is working) 
}
ISR(INT5_vect)
{
  itp_5++;
  PORTB &= 0b11111110; // turn off LED
}

void setup() {

  // setup an LED
  DDRB |= 0b00000001; //LED on PB0 (pin 53)
  
  // setup serial monitior
  Serial.begin(9600);     
  while (! Serial);
  Serial.println("Hello World!");

  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  cli(); //turn interrupts off during setup
  DDRE &= 0b11001111; // PE4 and 5 as inputs
  //EICRB |= 0b00000101;  // logical change trigger on INT4 and INT5 (table 15-3)
  EICRB |= 0b00001111; //rising edge trigger on INT4 and INT5
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5
  sei(); //turn interrupts on for use
  
}

void loop() {

  delay(100);
  Serial.print("ISR4: ");
  Serial.print(itp_4,DEC); 
  
  Serial.print("  ISR5: ");
  Serial.print(itp_5,DEC); 
  Serial.print("\n");

}
  
