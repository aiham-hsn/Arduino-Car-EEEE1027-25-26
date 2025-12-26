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
const uint8_t IN3 = 16;  // marked as pin A2 on the Arduino
const uint8_t IN4 = 17;  // marked as pin A3 on the Arduino
//const uint8_t IN3 = 1;  // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH
//const uint8_t IN4 = 0;  // DISCONNECT PINS FROM ARDUINO WHEN UPLOADING SKETCH

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t TOP_GEAR = 255;
const uint8_t CLIMB_GEAR = 148;
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

// Variables related to climbing the ramp, waiting, and spinning
const uint8_t MIN_RAMP_ANGLE = 23;
const unsigned long PLATFORM_ARRVE_TIME = 200;
const unsigned long PLATFORM_WAIT_TIME = 4000;
float maximumPitch = 0;
unsigned long ramp_climb_time = 0;
unsigned long reach_platform_time = 0;
bool climb_ramp = false;
bool on_platform = false;
bool has_waited = false;
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

  float zAccel = a.acceleration.z;
  struct carAngle carAngle = calculateAngle(a.acceleration.x, a.acceleration.y, zAccel);
  float currentPitch = carAngle.x;
  maximumPitch = max(maximumPitch, currentPitch);

  // Read values from the IR sensors
  unsigned int leftIRval = analogRead(IR_L);
  unsigned int rightIRval = analogRead(IR_R);

  // Check if the read values are less than the programmed threshold values
  // This means that not enough of the IR light from the IR LED is being detected by the photodiode, indictating that said sensor is above the black line
  bool rightIRdetect = (rightIRval < IRT_R) ? true : false;
  bool leftIRdetect = (leftIRval < IRT_L) ? true : false;

  if (climb_ramp == false) {
    printAngle(carAngle, maximumPitch);

    // Getting onto ramp
    if (maximumPitch <= MIN_RAMP_ANGLE) {
      printAngle(carAngle, maximumPitch);
      forward(CLIMB_GEAR);
    }

    // Fully on ramp
    if (maximumPitch > MIN_RAMP_ANGLE && zAccel <= 9) {
      printAngle(carAngle, maximumPitch);
      forward(CLIMB_GEAR);
    }

    // Getting off ramp
    if (maximumPitch > MIN_RAMP_ANGLE && zAccel > 9 && currentPitch < 2) {
      climb_ramp = true;
      ramp_climb_time = elapsed_time;
      printAngle(carAngle, maximumPitch);
    }
  }

  // Has not reached the middle of the ramp platform
  if (climb_ramp == true && on_platform == false && (elapsed_time - ramp_climb_time) < PLATFORM_ARRVE_TIME) {
    printAngle(carAngle, maximumPitch);
    forward(UPPER_GEAR);
  }

  // Has reached the middle of the ramp platform
  if (climb_ramp == true && on_platform == false && (elapsed_time - ramp_climb_time) >= PLATFORM_ARRVE_TIME) {
    on_platform = true;
    stop();
    printAngle(carAngle, maximumPitch);
    reach_platform_time = elapsed_time;
  }

  // Has not waited on the platform
  if (on_platform == true && has_waited == false && has_spun == false && (elapsed_time - reach_platform_time) < PLATFORM_WAIT_TIME) {
    stop();
    // printAngle(carAngle, maximumPitch);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Waiting 4s...");
  }
  // Has waited on the platform
  if (on_platform == true && has_waited == false && has_spun == false && (elapsed_time - reach_platform_time) >= PLATFORM_WAIT_TIME) {
    has_waited = true;
    stop();
    // printAngle(carAngle, maximumPitch);
  }

  if (on_platform == true && has_waited == true && has_spun == false) {
    // Put rotation code/funtion here
    rotateCar(REV_UPPER_GEAR, 2250);
    has_spun = true;
    stop();
    delay(1000);
  }

  if (has_spun == true) {
    printAngle(carAngle, maximumPitch);
  }

  //Serial.print("climb_ramp:");
  //Serial.print(climb_ramp);
  //Serial.print(", ");
  //Serial.print("on_platform:");
  //Serial.print(on_platform);
  //Serial.print(", ");
  //Serial.print("has_waited:");
  //Serial.print(has_waited);
  //Serial.print(", ");
  //Serial.print("has_spun:");
  //Serial.print(has_spun);
  //Serial.println("");

  //if (is_moving) {
  //  // Print time taken and distance travelled
  //  printTime(elapsed_time, 0, 0);
  //}

  delay(20);  // Delay for a small amount of time to smooth out operations
}


// ---------------------------------------
// ---------------------------------------
/*           HELPER FUNCTIONS           */
// ---------------------------------------
// ---------------------------------------


struct carAngle calculateAngle(float xAccel, float yAccel, float zAccel) {
  // Calculate tilt angles in degrees
  float angleX = atan2(yAccel, zAccel) * RAD_TO_DEG;
  float angleY = atan2(-xAccel, zAccel) * RAD_TO_DEG;

  // Normalize angles to a range of -90° to 90°
  if (angleX > 90) angleX = 180 - angleX;
  if (angleX < -90) angleX = -180 - angleX;
  if (angleY > 90) angleY = 180 - angleY;
  if (angleY < -90) angleY = -180 - angleY;

  struct carAngle angle = { angleX, angleY };
  return angle;
}

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

  while (degsRotated < 360 && (millis() - rotationStartTime) < MAX_ROTATION_TIME) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long timeOne = millis();
    float rawGyroZ = g.gyro.z * RAD_TO_DEG * -1.0;
    float deltaT = (float)(timeOne - timeZero) / (float)1000;
    // degsRotated += (rawGyroZ > 1) ? rawGyroZ * deltaT : 0;  // degs = degsPerSec * secs
    degsRotated += rawGyroZ * deltaT;  // degs = degsPerSec * secs
    timeZero = timeOne;

    // Exit while loop
    if (degsRotated > 360) {
      break;
    }

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

void printAngle(struct carAngle angle, float maxangle) {
  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  lcd.print(angle.x, 1);
  lcd.print("DEG  ");

  lcd.setCursor(0, 1);
  lcd.print("Max: ");
  lcd.print(maxangle, 1);
  lcd.print("DEG  ");
}
void printTime(unsigned long msecs, uint8_t col, uint8_t row) {
  const double secs = (double)msecs / (double)1000;
  lcd.setCursor(col, row);
  lcd.print("Time (s): ");
  lcd.print(secs, 1);  // print to 1 DP
}

void followLine(bool rightIRdetect, bool leftIRdetect) {
  // Neither of the IR sensors are seeing the black line
  if (!rightIRdetect && !leftIRdetect) {
    forward(81);
  }
  // Only the right IR sensor is seeing the black line
  if (rightIRdetect && !leftIRdetect) {
    right(118);
  }
  // Only the left IR sensor is seeing the black line
  if (!rightIRdetect && leftIRdetect) {
    left(118);
  }
  // Both IR sensors are seeing the black line
  if (rightIRdetect && leftIRdetect) {
    stop();
  }
}
void forward(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (is_moving == false) { is_moving = true; }
}
void right(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, REV_UPPER_GEAR);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (is_moving == false) { is_moving = true; }
}
void left(uint8_t speed) {
  analogWrite(ENA, REV_UPPER_GEAR);
  analogWrite(ENB, speed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (is_moving == false) { is_moving = true; }
}
void stop() {
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  if (is_moving == true) { is_moving = false; }
}
// void backward(uint8_t speed) {
//   analogWrite(ENA, speed);
//   analogWrite(ENB, speed);
//   digitalWrite(IN1, LOW);
//   digitalWrite(IN2, HIGH);
//   digitalWrite(IN3, HIGH);
//   digitalWrite(IN4, LOW);
//   if (is_moving == false) { is_moving = true; }
// }
