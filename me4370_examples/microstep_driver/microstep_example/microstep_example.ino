int PUL = 11; //define Pulse pin, PB5
int ENA = 23; //define Enable Pin, PA0
int DIR = 22; //define Direction pin, PA1
int dt=1000;

void setup() {

  DDRA  =  0b111111111; // Port A as output
  PORTA &= 0b111111100; // set enable low ('on') and direction low('cw')

  pinMode (PUL, OUTPUT);

}

void loop() {


  for (int i = 0; i < 6400; i++) //Forward 5000 steps
  {

    digitalWrite(PUL, HIGH);
    delayMicroseconds(dt);
    digitalWrite(PUL, LOW);
    //delayMicroseconds(50);
    delay(1);
  }

}
