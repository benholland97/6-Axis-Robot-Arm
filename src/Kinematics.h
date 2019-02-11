#ifndef KINEMATICS_H
#define KINEMATICS_H


#include <ArduinoSTL.h>
// #include <algorithm>
#include "Positions.h"
#include "DHParam.h"


class Kinematics {
public:
    Kinematics() {setup();};

    void setup();

	static JointAngles getNullPositionAngles();

    //calculate position from joint angles
    bool calcForwardKinematics(FullPosition& pos);
    //calculate joint angles from position 
    bool calcInverseKinematics(FullPosition& pos);


    // //full calculation of all potential solutions of joint angles to reach position
    // bool calcInverseKinematics(const FullPosition& pos, KinematicsSolutionType &recSolution, std::vector<KinematicsSolutionType> &validSolutions);
	// // // computes the configuration type of a given solution
	// // PoseConfigurationType computeConfiguration(const JointAngles angles);
	static float toRadians(float deg) {
            return deg*M_PI / 180.0; 
    }
    static float toDegrees(float rad) {
            return rad*180.0 / M_PI; 
    }

private:

	DHParam DHParams[NUM_SERVOS];
	// const int floatPrecisionDigits=8;
	// const float floatPrecision=pow(10.0,-floatPrecisionDigits);
	// TransMatrix TransMatrices[NUM_SERVOS];





};


#endif