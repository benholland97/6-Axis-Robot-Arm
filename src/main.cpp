#include <Arduino.h> 
#include "Config.h"
#include "ServoController.h"
#include "SerialRx.h"
#include "Kinematics.h"
#include "Positions.h"


ServoController* sc;
SerialRX* sRX;
// Kinematics* Kinematics;
Kinematics* kin;
FullPosition fp;
float angles[NUM_SERVOS];
float pos[NUM_SERVOS];


void setup() {
    Serial.begin(9600);
    Serial.println("Initialising Robot Arm");
    sc = new ServoController();
    kin = new  Kinematics();
    sRX = new SerialRX();
    fp = FullPosition();
    delay(50);
    float jAngles[6] = {0,0,0,0,0,0};
    FullPosition posFP = FullPosition(Point(190,0,220),Rotation(Kinematics::toRadians(0),Kinematics::toRadians(0),Kinematics::toRadians(0)));
    // FullPosition jointsFP = FullPosition(JointAngles(jAngles));

    // jointsFP.printContents();
    // posFP.printContents();
    // kin->calcForwardKinematics(jointsFP);
    // kin->calcInverseKinematics(posFP);
    // FullPosition testFP = FullPosition(posFP.angles);
    // kin->calcForwardKinematics(testFP);

    Serial.println("Forward kin - target:");
    // jointsFP.printContents();
    Serial.print("\n");

    // Serial.println("Inv kin - result:");
    // posFP.printContents();
    // Serial.print("\n\n");

    // if(sc->mechLimitsCheck(posFP.angles.getA())) {
        // Serial.println("Angle set go go go");
    // }

    
}

void setServoPos() {
    for(int i=0; i<NUM_SERVOS; ++i) {
        sc->setAngle(angles[i],i);
    }
}

float* toRadians(int size, float* x) {
    for(int i=0; i<size; ++i) {
        x[i] = x[i]*M_PI / 180.0; 
    }
    return x;
}


void loop() {
    // sc->setAngles(Kinematics::toRadians(testAngles));
    // if(sRX->recv(pos)) {
    //     Serial.println("\nData received");
    //     fp = FullPosition(Point(pos[0],pos[1],pos[2]),Rotation(Kinematics::toRadians(pos[3]),Kinematics::toRadians(pos[4]),Kinematics::toRadians(pos[5])));
    //     kin->calcInverseKinematics(fp);
    //     fp.printContents();
    //     // if(sc->mechLimitsCheck(fp.angles.getAnglesDeg())) {
    //         Serial.println("\nSetting new pos: \n");
    //         fp.printContents();
    //         sc->setAngles(fp.angles.getAnglesDeg());
    //     // } else {
    //     // Serial.println("\n Invalid position");
    //     // }
    //     // fp = FullPosition(pos);
    //     // kin ->calcForwardKinematics(fp);
    //     // fp.printContents();
    // } 
        
    delay(100);
}


