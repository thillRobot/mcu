//
//The example has been modified by Tristan Hill
//October 31, 2020 - Happy Halloween!
//
//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello, world!");
  lcd.setCursor(1,1);
  lcd.print("Extensive HD44780");
  lcd.setCursor(1,2);
  lcd.print("LiquidCrystal_I2C");
  lcd.setCursor(1,3);
  lcd.print("Arduino LCMIIC2004");
}


void loop()
{
}
