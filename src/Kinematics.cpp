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
    setup();
    // Serial.println("Calculating forward kinematics");
    // pos.printContents();
    TransMatrix T01 = TransMatrix(pos.angles[0],DHParams[0]);
    TransMatrix T12 = TransMatrix(-toRadians(90)+pos.angles[1],DHParams[1]);
    TransMatrix T23 = TransMatrix(pos.angles[2],DHParams[2]);
    TransMatrix T34 = TransMatrix(pos.angles[3],DHParams[3]);
    TransMatrix T45 = TransMatrix(pos.angles[4],DHParams[4]);
    TransMatrix T56 = TransMatrix(pos.angles[5],DHParams[5]);
    
    TransMatrix T06 = TransMatrix();
    // T01.printContent();
    // T12.printContent();
    // T23.printContent();
    // T34.printContent();
    // T45.printContent();
    // T56.printContent();

    T01 = T01 * T12 * T23;
    T34 = T34 * T45 * T56;
    T06 = T01 * T34;
    // T01.printContent();
    // T34.printContent();

    // T06 = T01 * T12 * T23 * T34 * T45 * T56;
    // // Serial.print("Forward t06");
    TransMatrix corRot = T06;
    TransMatrix::calcRotationMatrix(toRadians(-90),toRadians(-90),toRadians(-90),corRot);

    T06 = T06 * corRot;
    // T06.printContent();
    pos.position[0] = T06[0*N + 3];
    pos.position[1] = T06[1*N + 3];
    pos.position[2] = T06[2*N + 3];

    // extract rotation from homogeneous transform matrix
    //walter method
    //roll - alpha
    pos.orientation[0] =  atan2(T06[2*N+1],T06[2*N+2]);
    //pitch - beta
    pos.orientation[1] =  atan2(-T06[2*N+0],sqrt(T06[0*N+0]*T06[0*N+0] + T06[1*N+0]*T06[1*N+0]));
    float beta = pos.orientation[1];
    //yaw - gamma
    pos.orientation[2] =  atan2(T06[1*N+0],T06[0*N+0]);

    if(fabs(beta) == M_PI_2) {
        bool sign = beta > 0? 1:-1;
        pos.orientation[0] = 0;
        pos.orientation[2] = sign * atan2(T06[0*N+1],T06[1*N+1]);
    }

    //ar2 method

    // // y rotation - phi - pitch
    // pos.orientation[1] =  atan2(-T06[2*N+0],sqrt(T06[0*N+0]*T06[0*N+0] + T06[1*N+0]*T06[1*N+0]));
    // float cosPhi = cos(pos.orientation[1]);
    // //x rotation - psi - roll 
    // pos.orientation[0] =  atan2(T06[2*N+1]/cosPhi,T06[2*N+2]/cosPhi);
    // //z rotation - theta - yaw 
    // pos.orientation[2] = atan2(T06[1*N+0]/cosPhi,T06[0*N+0]/cosPhi);

    //https://www.mecademic.com/resources/Euler-angles/Euler-angles
    // // y rotation - phi - pitch
    // pos.orientation[1] =  asin(T06[0*N+2]);
    // //x rotation - psi - roll 
    // pos.orientation[0] =  atan2(-T06[1*N+2],T06[2*N+2]);
    // //z rotation - theta - yaw 
    // pos.orientation[2] = atan2(-T06[0*N+1],T06[0*N+0]);


    // //Temp rotation fix
    // for(int i=0; i<3; ++i) {
    //     float d = fabs(pos.orientation[i]) - M_PI;
    //     if((d < EPSILON) && (d > -EPSILON)) {
    //         pos.orientation[i] += pos.orientation[i] > 0? -M_PI:M_PI;
    //     }
    // }

    return true;
}

bool Kinematics::calcInverseKinematics(FullPosition& pos) {
    setup();
    //compute full homogenous transform matrix inc. rotation matrix
    // Serial.println("\nInput Tool Full Position :");
    // pos.printContents(); 
    // Serial.println("Inverse t06");
    float newAngles[NUM_SERVOS];

    //calc angle 0
    TransMatrix T06 = TransMatrix(pos);

    TransMatrix invCorRot = T06;
    TransMatrix::calcRotationMatrix(toRadians(90),toRadians(90),toRadians(90),invCorRot);
    // Serial.println("inv cor rot");
    // invCorRot.printContent();
    T06 = T06 * invCorRot;
    //calc WCP - equal to translation in -z axis by hand length
    // T06.printContent();

    float TCP_to_WCP[4] = {0,0,-WRIST_LENGTH,1};
    TransMatrix WCP = T06.multiply(4,1,TCP_to_WCP);
    // float wcp_x = pos.position.x - WRIST_LENGTH * T06[0*N + 2];
    // float wcp_y = pos.position.y - WRIST_LENGTH * T06[1*N + 2]; 
    // float wcp_z = pos.position.z - WRIST_LENGTH * T06[2*N + 2]; 
    Point wcp = Point(WCP[0],WCP[1],WCP[2]);
    // Serial.println("\nWrist Center Point :");
    // WCP.printContent();
    //TODO: Add second angle 0 solution  - pi + atan2(wcp.y,wcp.x)
    newAngles[0] = atan2(wcp[1],wcp[0]);
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
    newAngles[1] = M_PI_2 - (alpha + delta);
    newAngles[2] = M_PI_2 - gamma;

    //angles 3,4,5
    //avoid singularity x=0 y=0
    // if(fabs(wcp.x)< FLOAT_PRECISION) {
    //     wcp.x += EPSILON;
    // }   
    // if(fabs(wcp.y)< FLOAT_PRECISION) {
    //     wcp.y += EPSILON;
    // }

    TransMatrix T01 = TransMatrix(newAngles[0],DHParams[0]);
    TransMatrix T12 = TransMatrix(-toRadians(90)+newAngles[1],DHParams[1]);
    TransMatrix T23 = TransMatrix(newAngles[2],DHParams[2]);
    TransMatrix T03 = T01*T12*T23;
    // TransMatrix T03_inv = T03.inverse();
    // TransMatrix T03_inv = T03.rotInverse();
    TransMatrix T03_inv = T03.transpose();

    TransMatrix T36 = T03_inv * T06;
    float R36_22 = T36[2*N + 2];
    float R36_01 = T36[0*N + 1];
    float R36_12 = T36[1*N + 2];
    float R36_02 = T36[0*N + 2];
    float R36_21 = T36[2*N + 1];
    float R36_20 = T36[2*N + 0];

    if ((fabs(R36_22) > 1.0) && (fabs(R36_22) < (1.0+FLOAT_PRECISION))) {
		R36_22 = (R36_22>0)?1.0:-1.0;
	}

	if ((fabs(R36_01) > 1.0) && (fabs(R36_01) < (1.0+FLOAT_PRECISION))) {
		R36_01 = (R36_01>0)?1.0:-1.0;
	}

    newAngles[4] = acos(R36_22);
    float sinT4 = sin(newAngles[4]);
    //TODO: theta 4 = 0 gimbal lock solution
    if(fabs(newAngles[4]) < FLOAT_PRECISION) {
        Serial.println("Gimbal lock");
        newAngles[3] = 0;
        newAngles[5] = asin(-T06[0*N + 1]) - newAngles[3];
    } else {
        newAngles[3] = atan2(R36_12/sinT4,R36_02/sinT4);
        newAngles[5] = atan2(R36_21/sinT4,-R36_20/sinT4);
    }

    //temp angles pi error fix
    for(int i=3; i<6; ++i) {
        float d = fabs(newAngles[i]) - M_PI;
        if((d < EPSILON) && (d > -EPSILON)) {
            // Serial.print("Fixing pi error on angle");
            // Serial.println(i);
            newAngles[i] += newAngles[i] > 0? -M_PI:M_PI;
        }
    }
    //check against forward kin 
    // FullPosition tempPos = FullPosition(newAngles);
    // calcForwardKinematics(tempPos);
    // int errCount;
    // for(int i=0; i<NUM_SERVOS; ++i) {
    //     errCount += (newAngles[i] / tempPos.angles[i]);
    //     Serial.print("Ik angle ");
    //     Serial.print(i);
    //     Serial.print("\t : \t");
    //     Serial.println(newAngles[i]);
    //     Serial.print("FK angle ");
    //     Serial.print(i);
    //     Serial.print("\t : \t");
    //     Serial.println(tempPos.angles[i]);
    // }
    // Serial.print("Difference between inverse and forward kin:\t");
    // Serial.println(errCount);

    pos.angles.setAnglesRad(newAngles);


    return true;
}



JointAngles Kinematics::getNullPositionAngles() {
    return JointAngles::getDefaultPosition();
}
