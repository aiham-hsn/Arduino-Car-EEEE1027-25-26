#include <LiquidCrystal.h>

//IR Sensor pins
const uint8_t IR_L = 16;  // marked as pin A2 on the Arduino
const uint8_t IR_R = 17;  // marked as pin A3 on the Arduino

//IR Sensor threshold values
const uint8_t IRT_L = 100;
const uint8_t IRT_R = 125;


void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read values from the IR sensors
  unsigned int leftIRval = analogRead(IR_L);
  unsigned int rightIRval = analogRead(IR_R);

  // Check if the read values are less than the programmed threshold values
  // This means that not enough of the IR light from the IR LED is being detected by the photodiode, indictating that said sensor is above the black line
  bool rightIRdetect = (rightIRval < IRT_R) ? true : false;
  bool leftIRdetect = (leftIRval < IRT_L) ? true : false;

  Serial.print("R_IR val: ["); Serial.print(rightIRval); Serial.print("] | L_IR val: ["); Serial.print(leftIRval); Serial.println("]");
  // Serial.print("R_IR detect: ["); Serial.print(rightIRdetect); Serial.print("] | L_IR detect: ["); Serial.print(leftIRdetect); Serial.println("]");

}

