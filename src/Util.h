#ifndef KINEMATICS_H
#define KINEMATICS_H
#define M_PI           3.14159265358979323846


class Utils {
public:
    static float toRadians(float deg) {
            return deg*M_PI / 180.0; 
    }
    static float toDegrees(float rad) {
            return rad*180.0 / M_PI; 
    }
};


#endif