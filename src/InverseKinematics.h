#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

struct DHParamSingle {
    int idx;
    float r,alpha,d,theta;
    DHParamSingle(int _i,float _r,float _a, float _d):idx(_i),r(_r),alpha(_a),d(_d){}
};

struct DHTransMatrixSingle {
    int idx,row;
    float 
};

class InverseKinematics {
public:
    InverseKinematics();
private:

};


#endif