/*
  Hex Keypad Example Example

  Tennessee Technological University
  Tristan Hill - Jan 11, 2016 - Feb 11, 2019

  This example uses external pull down resistors 
  There is still an issue with 'row1' and the key mapping... needs fixing
*/

char key[2];
int del_t=3;

// the setup function runs once when you press reset or power the board
void setup() {

  // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial); 
  Serial.println("Hex Keypad Example");

  
  DDRA = 0b11110000;
  //PORTA = 0b11111111; // dont use internal PUs
  key[0]='-';
  key[1]=0;
}

// the loop function runs over and over again forever
void loop() {
        
  key[0]='-';
 
  delay(del_t);
  
  PORTA = 0b00010000;
  int my_state=PINA;
  if (my_state & 0b00000001)
  {
    key[0]='F'; // I dont know why this is not C
    // Col 1, Row 1 is Pressed
    Serial.println(my_state,BIN);
  }
  else if (PINA & 0b00000010)
  {
    key[0]='8';
    Serial.println(PINA,BIN);
    // Col 1, Row 2 is Pressed      
  }
  else if (PINA & 0b00000100)
  {
    key[0]='4';
    // Col 1, Row 3 is Pressed      
  }
  else if (PINA & 0b00001000)
  {
    key[0]='0';
    // Col 1, Row 4 is Pressed      
  }

  delay(del_t);
  
  PORTA = 0b00100000;
  if (PINA & 0b00000001)
  {
    key[0]='C'; // I dont know why this is not D
    // Col 2, Row 1 is Pressed     
  }
  else if (PINA & 0b00000010)
  {
    key[0]='9';
    //Col 2, Row 2 is Pressed     
  }
  else if (PINA & 0b00000100)
  {
    key[0]='5';
    //Col 2, Row 3 is Pressed  
  }
  else if (PINA & 0b00001000)
  {
    key[0]='1';
    // Col 2, Row 4 is Pressed   
  }
  
  delay(del_t);
  

  PORTA = 0b01000000;
  if (PINA & 0b00000001)
  {
    key[0]='D'; // I dont know why this is not D
    // Col 3, Row 1 is Pressed !");     
  }
  else if (PINA & 0b00000010)
  {
    key[0]='A';
    //Col 3, Row 2 is Pressed !");     
  }
  else if (PINA & 0b00000100)
  {
    key[0]='6';
    //Col 3, Row 3 is Pressed !");     
  }
  else if (PINA & 0b00001000)
  {
    key[0]='2';
    //Col 3, Row 4 is Pressed !");     
  }

  delay(del_t);
  
  PORTA = 0b10000000;
  if (PINA & 0b00000001)
  {
    key[0]='E'; // I dont know why this is not F
    // Col 4, Row 1 is Pressed !");     
  }
  else if (PINA & 0b00000010)
  {
    key[0]='B';
    //Col 4, Row 2 is Pressed !");     
  }
  else if (PINA & 0b00000100)
  {
    key[0]='7';
    //Col 4, Row 3 is Pressed !");     
  }
  else if (PINA & 0b00001000)
  {
    key[0]='3';
    //Col 4, Row 4 is Pressed !");     
  }

  delay(del_t);

  Serial.println(key);

}
