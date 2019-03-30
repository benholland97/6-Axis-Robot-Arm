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

bool ServoController::setAngles(float *pAngles) {
    for(int i=0; i<NUM_SERVOS; ++i) {
        if(!setAngle(pAngles[i],i)) {
            Serial.print("Failure on servo:");
            Serial.print(i);
            Serial.print("\t Angle :");
            Serial.println(pAngles[i]);
            return false;
        };
    }
    return true;
}

bool ServoController::setAngle(double angle, int servo) {
    if(boundsCheck(angle,servo)) {
        double pulselength;
        switch(servo) {
            case 0:
                pulselength = map(angle+SERVO0_OFFSET,angle_limits_min[0],angle_limits_max[0],SERVO0MIN, SERVO0MAX);
                break;
            case 1:
                pulselength = map(-angle+SERVO1_OFFSET,angle_limits_min[1],angle_limits_max[1],SERVO1MIN, SERVO1MAX);
                break;
            case 2:
                pulselength = map(angle+SERVO2_OFFSET,angle_limits_min[2],angle_limits_max[2],SERVO2MIN, SERVO2MAX);
                break;
            case 3: {
                // int iAngle = angle;
                // if(iAngle == -90) {
                //     angle = -80;
                //     // Serial.println("Fixing -90 rot angle 3 issue");
                // }
                pulselength = map(angle+SERVO3_OFFSET,angle_limits_min[3],angle_limits_max[3],SERVO3MIN, SERVO3MAX);
                break;
            }
            case 4:
                pulselength = map(angle,angle_limits_min[4],angle_limits_max[4],SERVO4MIN, SERVO4MAX);
                break;
            case 5:
                pulselength = map(angle+SERVO5_OFFSET,angle_limits_min[5],angle_limits_max[5],SERVO5MIN, SERVO5MAX);
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
    int iAngle = angle;
    if(iAngle > angle_limits_max[servo] || iAngle < angle_limits_min[servo]) {
        return false;
    }
    return true;
}


bool ServoController::mechLimitsCheck(float* pAngles) {
    for(int i=0; i<NUM_SERVOS; ++i) {
        if(!boundsCheck(pAngles[i],i)) {
            Serial.println("\n");
            Serial.println(pAngles[i]);
            Serial.println(i);
            return false;
        }
    }
    return true;
}

