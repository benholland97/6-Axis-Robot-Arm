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
    FullPosition fp = FullPosition(Point(0,150,0),Rotation(0,0,0));
    Serial.println("Pre inverse k calculations");
    for(int i=0; i<NUM_SERVOS; ++i) {
        Serial.print("ANGLES NUM : \t ");
        Serial.println(fp.angles[i]);
    }
    Serial.println("\n--------------------------\n");  

    kin->calcInverseKinematics(fp);

    Serial.println("After calc");
    for(int i=0; i<NUM_SERVOS; ++i) {
        Serial.print("ANGLES NUM : \t ");
        Serial.println(fp.angles[i]);
    }
    Serial.println("\n--------------------------\n");  
 
    
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