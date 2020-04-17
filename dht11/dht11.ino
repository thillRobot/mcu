#include <dht.h>
#include <LiquidCrystal.h>

//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

dht DHT;

#define DHT11_PIN 7

void setup(){
  //lcd.begin(16, 2);
    // setup serail monitior
  Serial.begin(9600);          //  setup serial
  while (! Serial);
  Serial.println("16-bit timer example");
}

void loop()
{
  int chk = DHT.read11(DHT11_PIN);
  
  /*
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Humidity: ");
  lcd.print(DHT.humidity);
  lcd.print("%");
  */
  Serial.print("Humidity: ");
  Serial.print(DHT.humidity);
  Serial.print("\n");
  delay(1000);
}

