#include "HelperFunctions.h"

struct carAngle calculateAngle(float xAccel, float yAccel, float zAccel) {
  // Calculate tilt angles in degrees
  float angleX = atan2(yAccel, zAccel) * RAD_TO_DEG;
  float angleY = atan2(-xAccel, zAccel) * RAD_TO_DEG;

  // Normalize angles to a range of -90° to 90°
  if (angleX > 90) angleX = 180 - angleX;
  if (angleX < -90) angleX = -180 - angleX;
  if (angleY > 90) angleY = 180 - angleY;
  if (angleY < -90) angleY = -180 - angleY;

  struct carAngle angle = {angleX, angleY};
  return angle;
}
void 360degRotation(uint8_t rotationSpeed) {
}
void rotate(uint8_t speed) {
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (is_moving == false) { is_moving = true; }
}

void printAngle(struct carAngle angle, uint8_t col, uint8_t row) {
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

