#include "ServoController.h"

ServoController::ServoController() {
    init(NUM_SERVOS);
}

ServoController::ServoController(int x) {
    init(x);
}

void ServoController::init(int x) {
    Serial.println("Initialising servo controller");
    pwm = Adafruit_PWMServoDriver();
    numServos = x;
    pwm.begin();
    pwm.setPWMFreq(50);  // Digital servos run at ~50 Hz updates
    delay(10);

}

bool ServoController::setAngle(double angle, int servo) {
    if(boundsCheck(angle,servo)) {
        double pulselength;
        switch(servo) {
            case 0:
                pulselength = map(angle,0,MG996R_MAX_ANGLE,SERVO0MIN, SERVO0MAX);
                break;
            case 1:
                pulselength = map(angle,0,MG996R_MAX_ANGLE,SERVO1MIN, SERVO1MAX);
                break;
            case 2:
                pulselength = map(angle,0,MG996R_MAX_ANGLE,SERVO2MIN, SERVO2MAX);
                break;
            case 3:
                pulselength = map(angle,MG90S_ELB_MIN_ANGLE,MG90S_ELB_MAX_ANGLE,SERVO3MIN, SERVO3MAX);
                break;
            case 4:
                pulselength = map(angle,0,MG90S_MAX_ANGLE,SERVO4MIN, SERVO4MAX);
                break;
            case 5:
                pulselength = map(angle,0,MG90S_MAX_ANGLE,SERVO5MIN, SERVO5MAX);
                break;
            default:
                pulselength = -1;
                break;
        }
        if (pulselength<0) {
            return false;
        }
        pwm.setPWM(servo,0,pulselength);
        return true;
    }
    return false;
}

// bool ServoController::setAngle(int rx[]) {
//     for(int i=0; i<numServos; ++i) {
//         double pos = mapRXToPulse(i,rx[i]);
//         if(boundsCheck(i,pos)) {
//             pwm.setPWM(i,0,pos);
//         }
//     }
//     return true;
// }

// double ServoController::mapRXToPulse(int i, double rx) {
//     return (rx-rx_limits_min[i])*((double)(pulse_limits_max[i] - pulse_limits_min[i])/(double)(rx_limits_max[i]-rx_limits_min[i]));
// }


int ServoController::getNumServos() {return numServos;}

bool ServoController::boundsCheck(double angle, int servo) {
    if(servo>numServos-1 || servo<0) {
        return false;
    }
    if(angle>angle_limits_max[servo] || angle <angle_limits_min[servo]) {
        return false;
    }
    return true;
}


