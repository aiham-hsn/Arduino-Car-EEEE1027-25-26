#include <LiquidCrystal.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#ifndef USER_HELPER_FUNCS_H
#define USER_HELPER_FUNCS_H

// Structs
struct carAngle {
  float x;
  float y;
};

//Function declarations
void printAngle(struct carAngle angle, uint8_t col, uint8_t row);
void printTime(unsigned long msecs, uint8_t col, uint8_t row);
struct carAngle calculateAngle(float xAccel, float yAccel, float zAccel);
void forward(uint8_t speed);
void backward(uint8_t speed);
void left(uint8_t speed);
void right(uint8_t speed);
void rotate(uint8_t speed);
void stop();

#endif
