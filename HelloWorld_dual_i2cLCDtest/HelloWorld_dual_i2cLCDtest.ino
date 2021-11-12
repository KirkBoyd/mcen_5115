//YWROBOT
//Compatible with the Arduino IDE 1.0
//Library version:1.1
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd1(0x21,20,4);  // set the lcd1 address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd2(0x27,20,4);  // set the lcd1 address to 0x27 for a 16 chars and 2 line display

void setup()
{
  lcd1.init();                      // initialize the lcd1 
  lcd1.init();
  // Print a message to the lcd1.
  lcd1.backlight();
  lcd1.setCursor(3,0);
  lcd1.print("Hello, world!");
  lcd1.setCursor(2,1);
  lcd1.print("thisIsArduino1");
   lcd1.setCursor(0,2);
  lcd1.print("Arduino LCM IIC 2004");
   lcd1.setCursor(2,3);
  lcd1.print("Power By Ec-yuan!");
  lcd2.init();                      // initialize the lcd1 
  lcd2.init();
  // Print a message to the lcd1.
  lcd2.backlight();
  lcd2.setCursor(3,0);
  lcd2.print("Hello, world!");
  lcd2.setCursor(2,1);
  lcd2.print("thisIsArduino2");
   lcd2.setCursor(0,2);
  lcd2.print("Arduino LCM IIC 2004");
   lcd2.setCursor(2,3);
  lcd2.print("Power By Ec-yuan!");
}


void loop()
{
}
