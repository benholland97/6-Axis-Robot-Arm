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

    bool setAngles(float* pAngles);

    bool setAngle(double angle, int servo);

    // bool setAngle(int pos[]);

    // int readAngle(int x);

    int getNumServos();

    bool mechLimitsCheck(float* angles);

    // double mapRXToPulse(int i,double rx);

private:
    void init(int x);
    bool boundsCheck(double angle, int servo);

    Adafruit_PWMServoDriver pwm;
    // int pins[NUM_SERVOS] = {PIN_SERVO0,PIN_SERVO1,PIN_SERVO2,PIN_SERVO3,PIN_SERVO4,PIN_SERVO5};
    // int pulse_limits_min[NUM_SERVOS] = {SERVO0MIN,SERVO1MIN,SERVO2MIN,SERVO3MIN,SERVO4MIN,SERVO5MIN};
    int pulse_limits_min[NUM_SERVOS] = {SERVO0MIN,SERVO0MIN,SERVO0MIN,SERVO0MIN,SERVO0MIN,SERVO0MIN};
    // int pulse_limits_max[NUM_SERVOS] = {SERVO0MAX,SERVO1MAX,SERVO2MAX,SERVO3MAX,SERVO4MAX,SERVO5MAX};
    int pulse_limits_max[NUM_SERVOS] = {SERVO0MAX,SERVO0MAX,SERVO0MAX,SERVO0MAX,SERVO0MAX,SERVO0MAX};
    // int angle_limits_min[NUM_SERVOS] = {-MG996R_MAX_ANGLE_2,-MG996R_MAX_ANGLE_2,-MG996R_MAX_ANGLE_2,MG90S_ELB_MIN_ANGLE,-MG90S_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2};
    int angle_limits_min[NUM_SERVOS] = {-MG996R_MAX_ANGLE_2,-MG996R_MAX_ANGLE_2,-MG996R_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2,-MG90S_MAX_ANGLE_2};    
    // int angle_limits_max[NUM_SERVOS] = {MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2,MG90S_ELB_MAX_ANGLE,MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2};
    int angle_limits_max[NUM_SERVOS] = {MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2,MG996R_MAX_ANGLE_2,MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2,MG90S_MAX_ANGLE_2};

    int numServos = 0;
};

#endif