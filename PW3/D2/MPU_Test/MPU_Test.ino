#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>

//Function declarations
float calculatePitch(float x, float y, float z);

// LCD setup pins
const uint8_t RS = 8;
const uint8_t EN = 9;
const uint8_t D4 = 4;
const uint8_t D5 = 5;
const uint8_t D6 = 6;
const uint8_t D7 = 7;


// Setup the LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// Setup the MPU
Adafruit_MPU6050 mpu;

void setup() {
  // Init LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Ramp Angle (DEG)");
}

void loop() {
  
}

float calculatePitch(float x, float y, float z) {
  float carPitch = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * RAD_TO_DEG;
  return carPitch;
}
