#include "Kinematics.h"


void Kinematics::setup() {
    //                      alpha               r                      d
    DHParams[0] = DHParam(toRadians(-90),   SHOULDER_OFFSET,      HIP_HEIGHT);
    DHParams[1] = DHParam(0,                HUMERUS_LENGTH,       0);
    DHParams[2] = DHParam(toRadians(-90),   0,                    0);
    DHParams[3] = DHParam(toRadians(90),    0,                    FOREARM_LENGTH);
    DHParams[4] = DHParam(toRadians(-90),   0,                    0);
    DHParams[5] = DHParam(0,                0,                    WRIST_LENGTH);
}

bool Kinematics::calcForwardKinematics(FullPosition& pos) {
    Serial.println("Calculating forward kinematics");
    TransMatrix T01 = TransMatrix(pos.angles[0],DHParams[0]);
    TransMatrix T12 = TransMatrix(-toRadians(90)+pos.angles[1],DHParams[1]);
    TransMatrix T23 = TransMatrix(pos.angles[2],DHParams[2]);
    TransMatrix T34 = TransMatrix(pos.angles[3],DHParams[3]);
    TransMatrix T45 = TransMatrix(pos.angles[4],DHParams[4]);
    TransMatrix T56 = TransMatrix(pos.angles[5],DHParams[5]);
    
    TransMatrix T06 = T01 * T12 * T23 * T34 * T45 * T56;
    // Serial.print("Forward t06");
    // T06.printContent();
    TransMatrix corRot = T06;
    TransMatrix::calcRotationMatrix(toRadians(-90),toRadians(-90),toRadians(-90),corRot);

    T06 = T06 * corRot;
    // T06.printContent();
    pos.position[0] = T06[0*N + 3];
    pos.position[1] = T06[1*N + 3];
    pos.position[2] = T06[2*N + 3];

    //extract rotation from homogeneous transform matrix
    pos.orientation[0] =  atan2(T06[2*N+1],T06[2*N+2]);
    pos.orientation[1] =  atan2(-T06[2*N+0],sqrt(T06[0*N+0]*T06[0*N+0] + T06[1*N+0]*T06[1*N+0]));
    pos.orientation[2] =  atan2(T06[1*N+0],T06[0*N+0]);

    return true;
}

bool Kinematics::calcInverseKinematics(FullPosition& pos) {
    //compute full homogenous transform matrix inc. rotation matrix
    // Serial.println("\nInput Tool Full Position :");
    // pos.printContents(); 
    // Serial.println("Inverse t06");
    //calc angle 0
    TransMatrix T06 = TransMatrix(pos);
    // T06.printContent();

    TransMatrix invCorRot = T06;
    TransMatrix::calcRotationMatrix(toRadians(90),toRadians(90),toRadians(90),invCorRot);
    // Serial.println("inv cor rot");
    // invCorRot.printContent();
    T06 = T06 * invCorRot;
    //calc WCP - equal to translation in -z axis by hand length
    float TCP_to_WCP[4] = {0,0,-WRIST_LENGTH,1};
    TransMatrix WCP = T06.multiply(4,1,TCP_to_WCP);
    // float wcp_x = pos.position.x - WRIST_LENGTH * T06[0*N + 2];
    // float wcp_y = pos.position.y - WRIST_LENGTH * T06[1*N + 2]; 
    // float wcp_z = pos.position.z - WRIST_LENGTH * T06[2*N + 2]; 
    Point wcp = Point(WCP[0],WCP[1],WCP[2]);
    // Serial.println("\nWrist Center Point :");
    // WCP.printContent();
    //TODO: Add second angle 0 solution  - pi + atan2(wcp.y,wcp.x)
    pos.angles[0] = atan2(wcp[1],wcp[0]);
    // pos.printContents();
    // Serial.println("\n");

    //angle 1 + 2
    //using triangle (joint1(A=base + hip height), joint2(c) and joint3(WCP)
    // a = humerus length, b = forearm length, c = distance wcp[x] to wcp[y]
    
    float zJ1ToWCP = wcp.z - HIP_HEIGHT;
    float lenBaseToWCPTop = sqrt(wcp.x*wcp.x + wcp.y*wcp.y) - SHOULDER_OFFSET;
    float c = sqrt(zJ1ToWCP*zJ1ToWCP + lenBaseToWCPTop*lenBaseToWCPTop);
    float a = FOREARM_LENGTH;
    float b = HUMERUS_LENGTH;
    float alpha = acos((a*a - b*b - c*c)/(-2*b*c));
    float gamma = acos((c*c - a*a - b*b)/(-2*a*b));
    float delta = atan2(zJ1ToWCP,lenBaseToWCPTop);
    //assumed elbow up orientation
    pos.angles[1] = M_PI_2 - (alpha + delta);
    pos.angles[2] = M_PI_2 - gamma;

    //angles 3,4,5
    //avoid singularity x=0 y=0
    if(fabs(wcp.x)< FLOAT_PRECISION) {
        wcp.x += EPSILON;
    }   
    if(fabs(wcp.y)< FLOAT_PRECISION) {
        wcp.y += EPSILON;
    }

    TransMatrix T01 = TransMatrix(pos.angles[0],DHParams[0]);
    TransMatrix T12 = TransMatrix(-toRadians(90)+pos.angles[1],DHParams[1]);
    TransMatrix T23 = TransMatrix(pos.angles[2],DHParams[2]);
    TransMatrix T03 = T01*T12*T23;
    TransMatrix T03_inv = T03.inverse();

    TransMatrix T36 = T03_inv * T06;
    float R36_22 = T36[2*N + 2];
    float R36_12 = T36[1*N + 2];
    float R36_02 = T36[0*N + 2];
    float R36_21 = T36[2*N + 1];
    float R36_20 = T36[2*N + 0];

    pos.angles[4] = acos(R36_22);
    float sinT4 = sin(pos.angles[4]);
    //TODO: theta 4 = 0 gimbal lock solution
    if(pos.angles[4] < FLOAT_PRECISION) {
        pos.angles[3] = 0;
        pos.angles[5] = asin(-T06[0*N + 1]) - pos.angles[3];
    } else {
        pos.angles[3] = atan2(R36_12/sinT4,R36_02/sinT4);
        pos.angles[5] = atan2(R36_21/sinT4,-R36_20/sinT4);
    }

    if(fabs(pos.angles[5]) == M_PI) {
        pos.angles[5] += pos.angles[5] > 0? -M_PI:M_PI;
    }
    if(fabs(pos.angles[3]) == M_PI) {
        pos.angles[3] += pos.angles[3] > 0? -M_PI:M_PI;
    }
    // JointAngles newJA = JointAngles(pos.angles);
    // Serial.println("\nForward kin gives angles: ");
    // testFP.printContents();
    



    // //c2 = (Wx^2 + Wy^2 + Wz^2 - a1^2 - d3^2) / 2*a1*d3 
    // float c2 = (pow(wcp.x,2) + pow(wcp.y,2) + pow(wcp.z,2) - pow(DHParams[1].getR(),2) - pow(DHParams[3].getD(),2)
    //     / 2*DHParams[1].getR()*DHParams[3].getD());
    // float s2 = sqrt(1-pow(c2,2));
    // pos.angles[2] = M_PI_2 + atan2(s2,c2);

    // pos.printContents();

    // float s1 = ((DHParams[1].getR()+DHParams[3].getD()*c2)*wcp.z - DHParams[3].getD()*s2*sqrt(pow(wcp.x,2) + pow(wcp.y,2)))
    //     / pow(wcp.x,2) + pow(wcp.y,2) + pow(wcp.z,2);
    // float c1 = ((DHParams[1].getR()+DHParams[3].getD()*c2)*sqrt(pow(wcp.x,2) + pow(wcp.y,2)) + DHParams[3].getD()*s2*wcp.z)
    //     /  pow(wcp.x,2) + pow(wcp.y,2) + pow(wcp.z,2);
    // pos.angles[1] = atan2(s1,c1);

    // //inverse kinematics spherical wrist 3 angles


    // TransMatrix T01 = TransMatrix(pos.angles[0],DHParams[0]);
    // TransMatrix T12 = TransMatrix(pos.angles[1],DHParams[1]);
    // TransMatrix T23 = TransMatrix(pos.angles[2],DHParams[2]);
    // TransMatrix T03 = (T01*=T12*=T23);
    // TransMatrix T36 = T03.inverse() *= T06;

    // float R36_3x = T36[0*N + 2];
    // float R36_3y = T36[1*N + 2];
    // float R36_3z = T36[2*N + 2];

    // pos.angles[4] = atan2(sqrt(pow(R36_3x,2)+pow(R36_3y,2)),R36_3z);
    // if(pos.angles[4] > 0) {
    //     pos.angles[3] = atan2(R36_3y,R36_3x);
    //     pos.angles[5] = atan2(T36[2*N+1], -T36[2*N+0]); 
    // } else {
    //     pos.angles[3] = atan2(-R36_3y,-R36_3x);
    //     pos.angles[4] *= -1;
    //     pos.angles[5] = atan2(-T36[2*N+1], T36[2*N+0]); 
    // }

    return true;
}



JointAngles Kinematics::getNullPositionAngles() {
    return JointAngles::getDefaultPosition();
}
