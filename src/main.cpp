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
float angles[NUM_SERVOS];


void setup() {
    Serial.begin(9600);
    Serial.println("Initialising Robot Arm");
    sc = new ServoController();
    kin = new  Kinematics();
    delay(50);
    FullPosition posFP = FullPosition(Point(134.35,134.45,220),Rotation(Kinematics::toRadians(0),Kinematics::toRadians(0),Kinematics::toRadians(45)));
    float jAngles[6] = {45,0,0,0,0,0};
    FullPosition jointsFP = FullPosition(JointAngles(jAngles));

    // jointsFP.printContents();
    kin->calcForwardKinematics(jointsFP);
    kin->calcInverseKinematics(posFP);

    Serial.println("Forward kin - target:");
    jointsFP.printContents();
    Serial.print("\n\n");

    Serial.println("Inv kin - result:");
    posFP.printContents();
    Serial.print("\n\n");

    
}

void setServoPos() {
    for(int i=0; i<NUM_SERVOS; ++i) {
        sc->setAngle(angles[i],i);
    }
}


void loop() {
    // if(sRX->recv(angles)) {
    //     setServoPos();
    // }


    delay(100);
}


