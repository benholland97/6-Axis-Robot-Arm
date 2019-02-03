#ifndef KINEMATICS_H
#define KINEMATICS_H

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