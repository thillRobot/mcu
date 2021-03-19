/*  
 *  ME4370 - Tennessee Technological University   
 *  Tristan Hill - 
 *
*/  

// interrupt counters
volatile uint8_t itp_4=0x00,itp_5=0x00;
volatile uint16_t counts=0;



// define the ISR functions
ISR(INT4_vect)
{
 
  bool chA = PINE&=0b00010000;
  bool chB = PINE&=0b00100000;
  
  if(chA&&(!chB)){
      counts++;
  }
  else if(chA&&chB){
      counts--;
  }
  else if((!chA)&&chB){
      counts++;
  } 
  else if ((!chA)&&(!chB)){
      counts--;
  }  

  itp_4++; 
  //PORTB |= 0b00000001; // turn on LED
  TIFR4 |= (1<<ICF4); //clear the flag (not sure this is working) 
}
ISR(INT5_vect)
{
  /*
  if((rise_B==1)&&((PINE&=0b00010000)==1)){
      Serial.print("c1");
      counts++;
      rise_A=0;
  }
  else if(rise_B==1){
    Serial.print("c2");
      counts--;
      rise_A=0;
  }
  else if((rise_B==0)&&((PINE&=0b00010000)==1)){
    Serial.print("c3");
      counts++;
      rise_A=1;
  } 
  else{
    Serial.print("c4");
      counts--;
      rise_A=1;
  }  
  Serial.print("  counts: ");
  Serial.print(counts,DEC); 
  Serial.print("\n");
  */
  //Serial.print("ISR B: ");
 // Serial.print(PINE,BIN); 
 // Serial.print("\n");
  itp_5++; 
  TIFR5 |= (1<<ICF5); //clear the flag (not sure this is working) 
}

void setup() {

  
  DDRB |= 0b00000000; //PB0 as input
  
  // setup serial monitior
  Serial.begin(9600);     
  while (! Serial);
  Serial.println("Hello World!");

  //setup external interrupts, INT4(PE4,pin2) and INT5(PE5,pin3)
  cli(); //turn interrupts off during setup
  DDRE &= 0b11001111; // PE4 and 5 as inputs
  EICRB |= 0b00000101;  // logical change trigger on INT4 and INT5 (table 15-3)
  //EICRB |= 0b00001111; //rising edge trigger on INT4 and INT5
  EIMSK |= 0b00110000; //external interrupt request enable on INT4 and INT5
  sei(); //turn interrupts on for use
  
}

void loop() {
/*
  delay(100);
  Serial.print("ISR4: ");
  Serial.print(itp_4,DEC); 
  
  Serial.print("  ISR5: ");
  Serial.print(itp_5,DEC); 
  Serial.print("\n");

  
  Serial.print("  counts: ");
  Serial.print(counts,DEC); 
  Serial.print("\n");
*/
  delay(100);
  Serial.print("counts: ");
  Serial.print(counts,DEC); 
  Serial.print("\n"); 
}
  
