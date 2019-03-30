#include <Arduino.h> 
#include "Config.h"
#include "ServoController.h"
#include "SerialRx.h"
#include "Kinematics.h"
#include "Positions.h"


// ServoController sc;
// SerialRX sRX;
// Kinematics* Kinematics;
// Kinematics kin;
// FullPosition fp;



float* toRadians(int size, float* x) {
    for(int i=0; i<size; ++i) {
        x[i] = x[i]*M_PI / 180.0; 
    }
    return x;
}

float* toDegrees(int size, float* x) {
    for(int i=0; i<size; ++i) {
        x[i] = x[i]*180 / M_PI; 
    }
    return x;
}

void setup() {
    Serial.begin(9600);
    Serial.println("Initialising Robot Arm");
    ServoController sc = ServoController();
    Kinematics kin =  Kinematics();
    SerialRX sRX = SerialRX(Serial);
    FullPosition fp = FullPosition();
    // float angles[NUM_SERVOS];
    // float pos[NUM_SERVOS];
    // float buffer[NUM_SERVOS];
    bool fKinFlag = false;
    bool fromComp = true;
    bool setMove = false;
    // delay(200);


    //main loop
    while(1) {
        while(setMove) {
            int max = 300;
            int min = 120;
            int y_rot = 0;
            int height = 220;
            for(int i=min; i<max; i+=2) {
                Serial.println(i);
                float newBuffer[NUM_SERVOS] = {i,y_rot,height,0,0,0};
                Point pos = Point(newBuffer[0],newBuffer[1],newBuffer[2]);
                toRadians(NUM_SERVOS,newBuffer);
                Rotation rot = Rotation(newBuffer[3],newBuffer[4],newBuffer[5]);
                fp = FullPosition(pos,rot);
                // Serial.println("Input position");
                // fp.printContents();
                kin.calcInverseKinematics(fp);
                // fp.printContents();
                sc.setAngles(fp.angles.getAnglesDeg());
                delay(20);
            }
            delay(100);
            Serial.println("Going back");
            for(int i=max; i>min; i-=2) {
                Serial.println(i);
                float newBuffer[NUM_SERVOS] = {i,-y_rot,height,0,0,0};
                Point pos = Point(newBuffer[0],newBuffer[1],newBuffer[2]);
                toRadians(NUM_SERVOS,newBuffer);
                Rotation rot = Rotation(newBuffer[3],newBuffer[4],newBuffer[5]);
                fp = FullPosition(pos,rot);
                // Serial.println("Input position");
                // fp.printContents();
                kin.calcInverseKinematics(fp);
                // fp.printContents();
                sc.setAngles(fp.angles.getAnglesDeg());
                delay(20);
            }
        }
        float buffer[NUM_SERVOS];
        if(sRX.recv(buffer)) {
            Serial.println("\nData received");
            if(fromComp) {
                // Serial.println("Angles received");
                Serial.println("Result of setting servos");
                bool res = sc.setAngles(buffer);
                Serial.println(res);
            }

            // } else {
            //     if(fKinFlag) {
            //     //input angles - forwrd kin
            //     fp = FullPosition(JointAngles(buffer));
            //     kin.calcForwardKinematics(fp);
            //     fp.printContents();
            //     } else {
            //         //input position - inverse kin
            //         Point pos = Point(buffer[0],buffer[1],buffer[2]);
            //         toRadians(NUM_SERVOS,buffer);
            //         Rotation rot = Rotation(buffer[3],buffer[4],buffer[5]);
            //         fp = FullPosition(pos,rot);
            //         Serial.println("Input position");
            //         fp.printContents();
            //         kin.calcInverseKinematics(fp);
            //         Serial.println("Final position");
            //         fp.printContents();

            //         JointAngles tempJA = JointAngles(fp.angles);
            //         FullPosition tempPos = FullPosition(tempJA);
            //         kin.calcForwardKinematics(tempPos);
            //         Serial.println("forward k check");
            //         tempPos.printContents();
            //     }

            // }

            // Serial.println("Result of setting servos");
            // bool res = sc.setAngles(fp.angles.getAnglesDeg());
            // Serial.println(res);
        }
    }











    // float jAngles[6] = {45,65,0,0,0,0};
    // FullPosition posFP = FullPosition(Point(209,0,10),Rotation(Kinematics::toRadians(0),Kinematics::toRadians(65),Kinematics::toRadians(0)));
    // FullPosition jointsFP = FullPosition(JointAngles(jAngles));

    // jointsFP.printContents();
    // posFP.printContents();
    // kin.calcForwardKinematics(jointsFP);
    // kin.calcInverseKinematics(posFP);
    // FullPosition testFP = FullPosition(posFP.angles);
    // kin.calcForwardKinematics(testFP);

    // Serial.println("Inv kin result:");
    // posFP.printContents();
    // Serial.print("\n");

    // Serial.println("Plugged back into forward k:");
    // testFP.printContents();
    // Serial.print("\n\n");

    // Serial.println("forward k:");
    // jointsFP.printContents();
    // Serial.print("\n\n");


    // if(sc.mechLimitsCheck(posFP.angles.getA())) {
        // Serial.println("Angle set go go go");
    // }

    
}

// void setServoPos() {
//     for(int i=0; i<NUM_SERVOS; ++i) {
//         sc.setAngle(angles[i],i);
//     }
// }




void loop() {
    // // sc.setAngles(Kinematics::toRadians(testAngles));
    // if(sRX.recv(pos)) {
    //     Serial.println("\nData received");
    //     fp = FullPosition(Point(pos[0],pos[1],pos[2]),Rotation(Kinematics::toRadians(pos[3]),Kinematics::toRadians(pos[4]),Kinematics::toRadians(pos[5])));
    //     kin.calcInverseKinematics(fp);
    //     fp.printContents();
    //     // if(sc.mechLimitsCheck(fp.angles.getAnglesDeg())) {
    //         Serial.println("\nSetting new pos: \n");
    //         fp.printContents();
    //         sc.setAngles(fp.angles.getAnglesDeg());
    //     // } else {
    //     // Serial.println("\n Invalid position");
    //     // }
    //     // fp = FullPosition(pos);
    //     // kin .calcForwardKinematics(fp);
    //     // fp.printContents();
    // } 
        
    delay(100);
}


