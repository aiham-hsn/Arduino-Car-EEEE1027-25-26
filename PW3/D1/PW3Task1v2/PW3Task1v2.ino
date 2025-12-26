#include <LiquidCrystal.h>

//Function declarations
void printDist(float dist, uint8_t col, uint8_t row);
void printTime(unsigned long msecs, uint8_t col, uint8_t row);
void forward(uint8_t speed);
void backward(uint8_t speed);
void left(uint8_t speed);
void right(uint8_t speed);
void stop();

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
const uint8_t IN3 = 18;  // marked as pin A5 on the Arduino
const uint8_t IN4 = 19;  // marked as pin A4 on the Arduino

// Motor enable pins
const uint8_t ENA = 11;
const uint8_t ENB = 3;

// Preset speeds that can be used to drive the motor at
const uint8_t TOP_GEAR = 255;
const uint8_t UPPER_GEAR = 128;
const uint8_t REV_UPPER_GEAR = 170;
const uint8_t NORMAL_GEAR = 80;
const uint8_t LOWER_GEAR = 50;

// Rotary Encoder pins
//const uint8_t ENC_L = 12;  // Not in use as pin 12 cannot be used for digitalPinToInterrupt()
const uint8_t ENC_R = 2;

//IR Sensor pins
const uint8_t IR_L = 16;  // marked as pin A2 on the Arduino
const uint8_t IR_R = 17;  // marked as pin A3 on the Arduino

//IR Sensor threshold values
const uint8_t IRT_L = 210;
const uint8_t IRT_R = 250;

// Variables related to counting pulses received from the rotary encoders
//volatile unsigned int countL = 0;  // Not in use, see comment on ENC_L
volatile uint16_t countR = 0;

// Define interrupt functions to be used to count the pulses from the encoders
//void count_left() { countL++; }  // Not in use, see comment on ENC_L
void count_right() {
  countR++;
}

// Variables related to time tracking, state keeping, and group num based delay
// unsigned long DURATION = 3 * 1000;  // Converted to milliseconds
const char GRP_PRFX[2] = "E"; 
const uint8_t GRP_NUM = 53;
const String GRP_NUM_TYPE = (GRP_NUM % 2) ? String("ODD") : String("EVEN");
const unsigned long DELAY_MSEC = (GRP_NUM % 2) ? 2000 : 3000;  // Delay is in milliseconds
const uint16_t DELAY_SEC = (uint16_t)DELAY_MSEC / (uint16_t)1000;
const float DELAY_DIST = (float)(GRP_NUM * 10);
unsigned long start_time = 0;
unsigned long elapsed_time = 0;
const unsigned long AUTOPILOT_ON = (25 * 1000);  // Amount of time after which the auto-stop functionality is enabled
bool is_moving = false;

// Variables related to printing and calculating distance travelled
float distance = 0;
const float CIRCUMFRNC = 26.748671;  // Circumference in cm ; Calculated from a diameter measurement of ~67mm
const uint8_t PPR = 40;              // Pulses per revolution, counted from the number of slits in the encoder wheel.
                                     // TODO: empirically measure the pulses generated per revolution
                                     // PPR empirically determined to be 40


// Setup the LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

void setup() {
  // Set encoder digital out pins as inputs to the Arduino
  //pinMode(ENC_L, INPUT);  // Not in use, see comment on ENC_L
  pinMode(ENC_R, INPUT);

  // Set the L298N's input pins as outputs from the Arduino
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Attach interrupts for counting the pulses
  //attachInterrupt(digitalPinToInterrupt(ENC_L), count_left, FALLING);  // Not in use, see comment on ENC_L
  attachInterrupt(digitalPinToInterrupt(ENC_R), count_right, FALLING);

  // Init LCD
  lcd.begin(16, 2);

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

  // Read values from the IR sensors
  unsigned int leftIRval = analogRead(IR_L);
  unsigned int rightIRval = analogRead(IR_R);

  // Check if the read values are less than the programmed threshold values
  // This means that not enough of the IR light from the IR LED is being detected by the photodiode, indictating that said sensor is above the black line
  bool rightIRdetect = (rightIRval < IRT_R) ? true : false;
  bool leftIRdetect = (leftIRval < IRT_L) ? true : false;

  // Calculate distance travelled
  //distance = ((float)((countL + countR) / 2) / (float)PPR) * CIRCUMFRNC;
  detachInterrupt(digitalPinToInterrupt(ENC_R));
  distance = ((float)countR / (float)PPR) * CIRCUMFRNC;
  attachInterrupt(digitalPinToInterrupt(ENC_R), count_right, FALLING);

  if (distance >= DELAY_DIST && distance < (DELAY_DIST + 0.8)) {
    stop();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Group ");
    lcd.print(GRP_PRFX);
    lcd.print(GRP_NUM);
    lcd.print(" : ");
    lcd.print(DELAY_SEC);
    lcd.print("s");
    printDist(distance, 0, 1);
    delay(DELAY_MSEC);
    lcd.clear();
  }
  if (!rightIRdetect && !leftIRdetect) {  // Neither of the IR sensors are seeing the black line
    forward(81);
  }
  if (rightIRdetect && !leftIRdetect) {  // Only the right IR sensor is seeing the black line
    right(118);
  }
  if (!rightIRdetect && leftIRdetect) {  // Only the left IR sensor is seeing the black line
    left(118);
  }
  if (rightIRdetect && leftIRdetect && elapsed_time > AUTOPILOT_ON) {  // Both IR sensors are seeing the black line
    stop();
  }
  if (is_moving) {
    // Print time taken and distance travelled
    printTime(elapsed_time, 0, 0);
    printDist(distance, 0, 1);
  }

  delay(34);  // Delay for a small amount of time to smooth out operations
}

void printDist(float dist, uint8_t col, uint8_t row) {
  lcd.setCursor(col, row);
  lcd.print("Dist (cm): ");
  lcd.print(dist, 1);  // print to 1 DP
}
void printTime(unsigned long msecs, uint8_t col, uint8_t row) {
  const double secs = (double)msecs / (double)1000;
  lcd.setCursor(col, row);
  lcd.print("Time (s): ");
  lcd.print(secs, 1);  // print to 1 DP
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
  if (elapsed_time > 20000 && elapsed_time < 24500) { delay(200); }
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
