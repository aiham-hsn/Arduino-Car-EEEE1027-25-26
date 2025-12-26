#include "HelperFunctions.h"

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
const uint8_t IN3 = 1;     // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
const uint8_t IN4 = 0;     // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t TOP_GEAR = 255;
const uint8_t UPPER_GEAR = 128;
const uint8_t REV_UPPER_GEAR = 170;
const uint8_t NORMAL_GEAR = 80;
const uint8_t LOWER_GEAR = 50;

//IR Sensor pins
const uint8_t IR_L = 16;  // marked as pin A2 on the Arduino
const uint8_t IR_R = 17;  // marked as pin A3 on the Arduino

//IR Sensor threshold values
const uint8_t IRT_L = 120;
const uint8_t IRT_R = 250;

// Variables related to time tracking, state keeping, and group num based delay
// unsigned long DURATION = 3 * 1000;  // Converted to milliseconds
unsigned long start_time = 0;
unsigned long elapsed_time = 0;
bool is_moving = false;
bool is_climbing = false;
bool has_climbed = false;


// Setup the MPU
Adafruit_MPU6050 mpu;
float maximumPitch = 0;

// Setup the LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {
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
    while (1) {
      delay(20);
    }
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MPU found");
  delay(750);

  // Set the accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  // Set the gyro range
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // Set the filter bandwidth
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);

  // Display run start message
  lcd.setCursor(0, 0);
  lcd.print("Beginning run...");
  delay(750);

  // Start run & record start time
  lcd.clear();
  start_time = millis();
}

void loop() {
  // Calculate elapsed time
  elapsed_time = millis() - start_time;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  struct carAngle carAngle = calculateAngle(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  maximumPitch = max(maximumPitch, carAngle.x);

  // Read values from the IR sensors
  unsigned int leftIRval = analogRead(IR_L);
  unsigned int rightIRval = analogRead(IR_R);

  // Check if the read values are less than the programmed threshold values
  // This means that not enough of the IR light from the IR LED is being detected by the photodiode, indictating that said sensor is above the black line
  bool rightIRdetect = (rightIRval < IRT_R) ? true : false;
  bool leftIRdetect = (leftIRval < IRT_L) ? true : false;

  if (carAngle.x > 1.8) {
    is_climbing = true;
  }

  // Neither of the IR sensors are seeing the black line
  if (!has_climbed && !rightIRdetect && !leftIRdetect) {
    if (is_climbing) { forward(UPPER_GEAR); } else { forward(81); }
  }
  // Only the right IR sensor is seeing the black line
  if (!has_climbed && rightIRdetect && !leftIRdetect) {
    right(118);
  }
  // Only the left IR sensor is seeing the black line
  if (!has_climbed && !rightIRdetect && leftIRdetect) {
    left(118);
  }
  // Both IR sensors are seeing the black line
  if (!has_climbed && rightIRdetect && leftIRdetect && elapsed_time > AUTOPILOT_ON) {
    stop();
  }

  if (is_climbing) {
    printAngle();
  }

  //if (is_moving) {
  //  // Print time taken and distance travelled
  //  printTime(elapsed_time, 0, 0);
  //}

  delay(34);  // Delay for a small amount of time to smooth out operations
}

