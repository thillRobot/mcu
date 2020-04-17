/*
    ME4370 - Tennessee Technological University
    This program will track the position of a DC motor
    Using the an optical encoder
    Note, we are not doing full quad at this point

    Capture on rising edges of ChA, check dir. on chB and update position
    accordinginly
      CHA on PE4, CHB on PE5, intent is to capture on rising edges of
      CHA for the moment
    Externally Trigger Interrupt Service Routines (timer capture)

*/

// interrupt counters
volatile int32_t encoder_cts = 0, ctr_4 = 0, ctr_5 = 0;

// define the ISR functions
ISR(INT4_vect)
{
  if ((PINE & 0b00100000) == 0b00100000) encoder_cts++; // check if CH5 is leading CH4, if so, forward and inc
  else encoder_cts--; // else decrement
  ctr_4++;
}
ISR(INT5_vect)
{
  if ((PINE & 0b00010000) == 0b00000000) encoder_cts++; // check if CH4 is lagging Ch5, if so forward and inc
  else encoder_cts--; // else decrement
  ctr_5++;
}

void setup() {

  // setup serial monitior
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Hello World!");

  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  DDRE &= 0b11001111; // PE4 and 5 as inputs
  EICRB |= 0b00001111; //rising edge trigger on INT4 and INT5
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5
  EIFR |= 0b00110000; // clear flag
  
}

void loop() {


  Serial.print("encoder_cts: ");
  Serial.print(encoder_cts, DEC);

  Serial.print("  ISR4: ");
  Serial.print(ctr_4, DEC);
    Serial.print("  ISR5: ");
  Serial.print(ctr_5, DEC);
  Serial.print("\n");
  delay(100);

}

