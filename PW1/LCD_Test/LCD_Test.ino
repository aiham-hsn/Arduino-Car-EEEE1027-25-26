#include <LiquidCrystal.h>

// LCD setup pins
const byte RS = 8;
const byte EN = 9;
const byte D4 = 4;
const byte D5 = 5;
const byte D6 = 6;
const byte D7 = 7;

// Setup the LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup()
{
  lcd.begin(16, 2);
  lcd.print("Hello, World!");
}

void loop()
{
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
}

