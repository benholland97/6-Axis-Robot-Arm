#ifndef Kinematics_H
#define Kinematics_H

#include "DHParam.h"


class Kinematics {
public:
    Kinematics();

    void setup();

    void calcForwardKinematics();

    void calcInverseKinematics();
private:

};


#endif