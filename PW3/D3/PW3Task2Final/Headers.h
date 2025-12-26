#ifndef USER_FUNCS_H
#define USER_FUNCS_H

// Structs
struct carAngle {
  float x;
  float y;
};

//Function declarations
struct carAngle calculateAngle(float xAccel, float yAccel, float zAccel);
void rotateCar(uint8_t rotationSpeed, unsigned long MAX_ROTATION_TIME);
void commandRotate(uint8_t speed);
void commandRotateLeft(uint8_t speed);
void printAngle(struct carAngle angle, float maxangle);
void printTime(unsigned long msecs, uint8_t col, uint8_t row);
void followLine();
void forward(uint8_t speed);
void backward(uint8_t speed);
void left(uint8_t speed);
void right(uint8_t speed);
void stop();

#endif
