#include "Kinematics.h"


void Kinematics::setup() {
    //                      alpha               r                      d
    DHParams[0] = DHParam(toRadians(-90),   SHOULDER_LENGTH,      SHOULDER_HEIGHT);
    DHParams[1] = DHParam(0,                HUMERUS_LENGTH,       0);
    DHParams[2] = DHParam(toRadians(-90),   0,                    0);
    DHParams[3] = DHParam(toRadians(90),    0,                    FOREARM_LENGTH);
    DHParams[4] = DHParam(toRadians(-90),   0,                    0);
    DHParams[5] = DHParam(0,                0,                    WRIST_LENGTH);

    // TransMatrices[0] = TransMatrix()

}

bool Kinematics::calcInverseKinematics(FullPosition& pos) {
    // //1. Compute angle 0
    // //set transformation matrix T06
    // TransMatrix T06 = TransMatrix(pos);
    // //set WCP relative to TCP tranlation matrix
    // float TCP_to_WCP[4] = {0,0,-WRIST_LENGTH,1};
    // //calculate WCP 
    // TransMatrix WCP = T06.multiply(4,1,TCP_to_WCP);
    // //calc angle 0 - forward/backward
    // float angle0_1 = atan2(WCP[0], WCP[1]);
    // float angle0_2 = atan2(-WCP[0], -WCP[1]);
    // //test for above origin singularity
    // if((fabs(WCP[0])< FLOAT_PRECISION) && (fabs(WCP[1]) < FLOAT_PRECISION)) {
    //     angle0_1 = current[0];
	// 	angle0_solution2 = HALF_PI - current[0];
    // }


    //compute full homogenous transform matrix inc. rotation matrix 
    TransMatrix T06 = TransMatrix(pos);
    //calc WCP 
    float wcp_x = pos.position.x - WRIST_LENGTH * T06[0*NUM_MATRIX_COLUMNS + 2];
    float wcp_y = pos.position.y - WRIST_LENGTH * T06[1*NUM_MATRIX_COLUMNS + 2]; 
    float wcp_z = pos.position.z - WRIST_LENGTH * T06[2*NUM_MATRIX_COLUMNS + 2]; 

    Point wcp = Point(wcp_x,wcp_y,wcp_z);
    //TODO: Add second angle 0 solution  - pi + atan2(wcp.y,wcp.x)
    pos.angles[0] = atan2(wcp.y,wcp.x);

    //c2 = (Wx^2 + Wy^2 + Wz^2 - a1^2 - d3^2) / 2*a1*d3 
    float c2 = (pow(wcp.x,2) + pow(wcp.y,2) + pow(wcp.z,2) - pow(DHParams[1].getR(),2) - pow(DHParams[3].getD(),2)
        / 2*DHParams[1].getR()*DHParams[3].getD());
    float s2 = sqrt(1-pow(c2,2));
    pos.angles[2] = M_PI_2 + atan2(s2,c2);
    
    float s1 = ((DHParams[1].getR()+DHParams[3].getD()*c2)*wcp.z - DHParams[3].getD()*s2*sqrt(pow(wcp.x,2) + pow(wcp.y,2)))
        / pow(wcp.x,2) + pow(wcp.y,2) + pow(wcp.z,2);
    float c1 = ((DHParams[1].getR()+DHParams[3].getD()*c2)*sqrt(pow(wcp.x,2) + pow(wcp.y,2)) + DHParams[3].getD()*s2*wcp.z)
        /  pow(wcp.x,2) + pow(wcp.y,2) + pow(wcp.z,2);
    pos.angles[1] = atan2(s1,c1);

    //inverse kinematics spherical wrist 3 angles

    //avoid singularity x=0 y=0
    if(fabs(wcp.x)< FLOAT_PRECISION) {
        wcp.x += EPSILON;
    }   
    if(fabs(wcp.y)< FLOAT_PRECISION) {
        wcp.y += EPSILON;
    }
    TransMatrix T01 = TransMatrix(pos.angles[0],DHParams[0]);
    TransMatrix T12 = TransMatrix(pos.angles[1],DHParams[1]);
    TransMatrix T23 = TransMatrix(pos.angles[2],DHParams[2]);
    TransMatrix T03 = (T01*=T12*=T23);
    TransMatrix T36 = T03.inverse() *= T06;

    float R36_3x = T36[0*NUM_MATRIX_COLUMNS + 2];
    float R36_3y = T36[1*NUM_MATRIX_COLUMNS + 2];
    float R36_3z = T36[2*NUM_MATRIX_COLUMNS + 2];

    pos.angles[4] = atan2(sqrt(pow(R36_3x,2)+pow(R36_3y,2)),R36_3z);
    if(pos.angles[4] > 0) {
        pos.angles[3] = atan2(R36_3y,R36_3x);
        pos.angles[5] = atan2(T36[2*NUM_MATRIX_COLUMNS+1], -T36[2*NUM_MATRIX_COLUMNS+0]); 
    } else {
        pos.angles[3] = atan2(-R36_3y,-R36_3x);
        pos.angles[4] *= -1;
        pos.angles[5] = atan2(-T36[2*NUM_MATRIX_COLUMNS+1], T36[2*NUM_MATRIX_COLUMNS+0]); 
    }

    return true;
}

JointAngles Kinematics::getNullPositionAngles() {
    return JointAngles::getDefaultPosition();
}
