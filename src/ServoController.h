#ifndef SERVOCONTROLLER_H
#define SERVOCONTROLLER_H

// #include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
// #include <HardwareSerial.h>
#include "Config.h"


class ServoController {
public:

    ServoController();

    ServoController(int x);

    // bool testAttachment();

    bool setAngle(double angle, int servo);

    // bool setAngle(int pos[]);

    // int readAngle(int x);

    int getNumServos();

    // double mapRXToPulse(int i,double rx);

private:
    void init(int x);
    bool boundsCheck(double angle, int servo);

    Adafruit_PWMServoDriver pwm;
    int pins[NUM_SERVOS] = {PIN_SERVO0,PIN_SERVO1,PIN_SERVO2,PIN_SERVO3,PIN_SERVO4,PIN_SERVO5};
    int pulse_limits_min[NUM_SERVOS] = {SERVO0MIN,SERVO1MIN,SERVO2MIN,SERVO3MIN,SERVO4MIN,SERVO5MIN};
    int pulse_limits_max[NUM_SERVOS] = {SERVO0MAX,SERVO1MAX,SERVO2MAX,SERVO3MAX,SERVO4MAX,SERVO5MAX};
    int angle_limits_min[NUM_SERVOS] = {0,0,0,MG90S_ELB_MIN_ANGLE,0,0};
    int angle_limits_max[NUM_SERVOS] = {MG996R_MAX_ANGLE,MG996R_MAX_ANGLE,MG996R_MAX_ANGLE,MG90S_ELB_MAX_ANGLE,MG90S_MAX_ANGLE,MG90S_MAX_ANGLE};
    int numServos = 0;
};

#endif