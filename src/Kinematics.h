#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "DHParam.h"
#include "Positions.h"
#include <ArduinoSTL.h>


// a configuration is one valid solution of the inverse kinematics problem. There are 8 solutions
// max., not necessarily all valid all the time. Different solutions can be obtained when
// considering the bot to look forward or backward (hip joint), flipping or not flipping the triangle
// as defined by angles 1,2, and 3; or by considering different the forearm being upwards or downwards.
struct PoseConfigurationType {
	// types or arm position
	enum PoseDirectionType {FRONT, BACK }; 	// look to front or to the back (axis 0)
	enum PoseFlipType{ FLIP, NO_FLIP}; 		// elbow axis is above or below
	enum PoseForearmType{ UP, DOWN}; 		// elbow axis is above or below

	PoseDirectionType poseDirection;
	PoseFlipType poseFlip;
	PoseForearmType poseTurn;

	bool operator==(const PoseConfigurationType par) {
		return ((poseDirection == par.poseDirection) &&
				(poseFlip== par.poseFlip) &&
				(poseTurn == par.poseTurn));
	}
	bool operator!=(const PoseConfigurationType par) {
		return (!((*this) == par));
	}
};

// A solution is determined by a configuration and a set of angles
class KinematicsSolutionType {
public:
	KinematicsSolutionType () {};
	KinematicsSolutionType (const KinematicsSolutionType& par) { config = par.config;angles = par.angles; };
	void operator=(const KinematicsSolutionType& par) { config = par.config;angles = par.angles; };

	PoseConfigurationType config;
	JointAngles angles;
};


class Kinematics {
public:
    Kinematics();

    void setup();

    //calculate position from joint angles
    bool calcForwardKinematics(FullPosition& pos);
    //calculate joint angles from position - assumes current angles represent correct solution
    bool calcInverseKinematics(FullPosition& pos);
    //full calculation of all potential solutions of joint angles to reach position
    bool calcInverseKinematics(const FullPosition& pos, KinematicsSolutionType &solutions, std::vector<KinematicsSolutionType> &validSolution);



private:

};


#endif