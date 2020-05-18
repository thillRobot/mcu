/*
  Hex Keypad Example Example

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016
   
*/

char key[2];

// the setup function runs once when you press reset or power the board
void setup() {

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("Hex Keypad Example");

  //DDRB = 0b11111111;
  DDRA = 0b11110000;
  //PORTA = 0b11111111; // dont use PUs

  key[1]=0;
}

// the loop function runs over and over again forever
void loop() {

  
  //delay(50);
  //Serial.println(PINA,BIN);         
  
  PORTA = 0b00010000;
  for(int i=0;i<5;i++)
  {
    if (PINA & 0b00000001)
    {
      key[0]='C';
      //Serial.println(" Col 1, Row 1 is Pressed !");     
    }
    else if (PINA & 0b00000010)
    {
      key[0]='8';
      //Serial.println(" Col 1, Row 2 is Pressed !");     
    }
    else if (PINA & 0b00000100)
    {
      key[0]='4';
      //Serial.println(" Col 1, Row 3 is Pressed !");     
    }
    else if (PINA & 0b00001000)
    {
      key[0]='0';
      //Serial.println(" Col 1, Row 4 is Pressed !");     
    }
    delay(25);
  }
  delay(25);
  
  for(int i=0;i<5;i++)
  {
    PORTA = 0b00100000;
    if (PINA & 0b00000001)
    {
      key[0]='D';
      //Serial.println(" Col 2, Row 1 is Pressed !");     
    }
    else if (PINA & 0b00000010)
    {
      key[0]='9';
      //Serial.println(" Col 2, Row 2 is Pressed !");     
    }
    else if (PINA & 0b00000100)
    {
      key[0]='5';
      //Serial.println(" Col 2, Row 3 is Pressed !");     
    }
    else if (PINA & 0b00001000)
    {
      key[0]='1';
      //Serial.println(" Col 2, Row 4 is Pressed !");     
    }
    delay(25);
  }
  delay(25);
  
  for(int i=0;i<5;i++)
  {
    PORTA = 0b01000000;
    if (PINA & 0b00000001)
    {
      key[0]='E';
      //Serial.println(" Col 3, Row 1 is Pressed !");     
    }
    else if (PINA & 0b00000010)
    {
      key[0]='A';
      //Serial.println(" Col 3, Row 2 is Pressed !");     
    }
    else if (PINA & 0b00000100)
    {
      key[0]='6';
      //Serial.println(" Col 3, Row 3 is Pressed !");     
    }
    else if (PINA & 0b00001000)
    {
      key[0]='2';
      //Serial.println(" Col 3, Row 4 is Pressed !");     
    }
    delay(25);
  }
  delay(25);
  
  for(int i=0;i<5;i++)
  {
    PORTA = 0b10000000;
    if (PINA &= 0b00000001)
    {
      key[0]='F';
      //Serial.println(" Col 4, Row 1 is Pressed !");     
    }
    else if (PINA & 0b00000010)
    {
      key[0]='B';
      //Serial.println(" Col 4, Row 2 is Pressed !");     
    }
    else if (PINA & 0b00000100)
    {
      key[0]='7';
      //Serial.println(" Col 4, Row 3 is Pressed !");     
    }
    else if (PINA & 0b00001000)
    {
      key[0]='3';
      //Serial.println(" Col 4, Row 4 is Pressed !");     
    }
    delay(25);
  }
  delay(25);

  //std::sprintf
  Serial.println(key);

}
