#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Headers.h"

// LCD setup pins
const uint8_t RS = 8;
const uint8_t EN = 9;
const uint8_t D4 = 4;
const uint8_t D5 = 5;
const uint8_t D6 = 6;
const uint8_t D7 = 7;

// Input pins to drive the motors
const uint8_t IN1 = 13;
const uint8_t IN2 = 12;
//const uint8_t IN3 = 18;  // marked as pin A4 on the Arduino
//const uint8_t IN4 = 19;  // marked as pin A5 on the Arduino
//const uint8_t IN3 = 1;  // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
//const uint8_t IN4 = 0;  // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
const uint8_t IN3 = 16;  // marked as pin A2 on the Arduino
const uint8_t IN4 = 17;  // marked as pin A3 on the Arduino

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t TOP_GEAR = 255;
const uint8_t UPPER_GEAR = 128;
const uint8_t REV_UPPER_GEAR = 170;
const uint8_t NORMAL_GEAR = 80;
const uint8_t LOWER_GEAR = 50;

unsigned long start_time = 0;
unsigned long elapsed_time = 0;
bool has_spun = false;


// Setup the MPU
Adafruit_MPU6050 mpu;

// Setup the LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {
  Serial.begin(115200);
  // Set the L298N's input pins as outputs from the Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Init LCD
  lcd.begin(16, 2);

  if (!mpu.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("MPU not found");
    while (1) { delay(20); }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU found");
  delay(750);

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  delay(100);

  // Display run start message
  lcd.setCursor(0, 0);
  lcd.print("Testing 360...");
  delay(1000);

  // Start run & record start time
  lcd.clear();
  start_time = millis();
}

void loop() {
  // Calculate elapsed time
  elapsed_time = millis() - start_time;

  if (has_spun == false) {
    // Put rotation code/funtion here
    rotateCar(REV_UPPER_GEAR, 2250);
    has_spun = true;
    delay(1000);
  }

  delay(20);  // Delay for a small amount of time to smooth out operations
}


// ---------------------------------------
// ---------------------------------------
/*           HELPER FUNCTIONS           */
// ---------------------------------------
// ---------------------------------------


void rotateCar(uint8_t rotationSpeed, unsigned long MAX_ROTATION_TIME) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Rotating for");
  lcd.setCursor(0, 1);
  lcd.print("360 degrees");

  // Timekeeping for dT
  unsigned long timeZero = millis();
  unsigned long rotationStartTime = millis();

  float degsRotated = 0;

  commandRotate(rotationSpeed);

  while (degsRotated < 360) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long timeOne = millis();
    float rawGyroZ = g.gyro.z * RAD_TO_DEG;
    float deltaT = (float)(timeOne - timeZero) / (float)1000;
    degsRotated += rawGyroZ * deltaT * -1.0;  // degs = degsPerSec * secs
    timeZero = timeOne;
    Serial.print("degsRotated:");
    Serial.println(degsRotated);

    // Exit while loop
    if (degsRotated > 350) { break; }

    delay(10);
  }
  // Stop car from rotating
  stop();
}
void commandRotate(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void commandRotateLeft(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
