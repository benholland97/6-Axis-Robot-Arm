#include "ServoController.h"

ServoController::ServoController() {
    init(NO_ACTUATORS);
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
}

bool ServoController::setAngles(JointAngles ja) {
    for(int i=0; i<(int)ja.angles.size(); ++i) {
        if(!setAngle(ja.angles[i],i)) {
            cout<<"Failure setting angle "<<ja.angles[i]<<" to servo "<<i;
            return false;
        };
    }
    return true;
}

bool ServoController::setAngle(double angle, int servo) {
    if(boundsCheck(angle,servo)) {
        double pulselength;
        double offset;
        switch(servo) {
            case 0:
                pulselength = map(angle+SERVO0_OFFSET,angle_limits_min[0],angle_limits_max[0],SERVO0MIN, SERVO0MAX);
                offset = pulselength - 289;
                break;
            case 1:
                pulselength = map(-angle+SERVO1_OFFSET,angle_limits_min[1],angle_limits_max[1],SERVO1MIN, SERVO1MAX);
                offset = pulselength - 237;
                break;
            case 2:
                pulselength = map(angle+SERVO2_OFFSET,angle_limits_min[2],angle_limits_max[2],SERVO2MIN, SERVO2MAX);
                offset = pulselength - 277;
                break;
            case 3: 
                pulselength = map(angle+SERVO3_OFFSET,angle_limits_min[3],angle_limits_max[3],SERVO3MIN, SERVO3MAX);
                offset = pulselength - 277;
                break;
            case 4:
                pulselength = map(angle,angle_limits_min[4],angle_limits_max[4],SERVO4MIN, SERVO4MAX);
                offset = pulselength - 267;                
                break;
            case 5:
                pulselength = map(angle+SERVO5_OFFSET,angle_limits_min[5],angle_limits_max[5],SERVO5MIN, SERVO5MAX);
                offset = pulselength - 289;                
                break;
            default:
                pulselength = -1;
                break;
        }
        if (pulselength<0) {
            return false;
        }
        cout<<"Offset :"<<offset<<" Servo: "<<servo<<"\n";
        pwm.setPWM(servo,0,pulselength);
        return true;
    }
    return false;
}


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
    for(int i=0; i<NO_ACTUATORS; ++i) {
        if(!boundsCheck(pAngles[i],i)) {
            return false;
        }
    }
    return true;
}

void ServoController::addTargetPosition(float* _a){
    targetAngles.push_back(JointAngles(_a));
}

void ServoController::addTargetPosition(JointAngles _ja){
    targetAngles.push_back(_ja);
}

bool ServoController::moveOne() {
    // cout<<"Target angles "<< targetAngles.front();
    bool ret = setAngles(targetAngles.front());
    // cout<<"\tResult :"<<ret<<"\n";
    targetAngles.pop_front();
    return ret;
}


ostream& operator<<(ostream& os, const JointAngles& _ja) {
    JointAngles ja(_ja);
    os <<"{ 0: " <<ja[0] << "\t 1: " << ja.getAngleDeg(1) << "\t 2: "<< ja.getAngleDeg(2) <<"\t 3: "
        <<ja.getAngleDeg(3)<<"\t 4: "<<ja.getAngleDeg(4)<<"\t 5: "<<ja.getAngleDeg(5)<<" } ";
	return os;
}


