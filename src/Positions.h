#ifndef POSITIONS_H
#define POSITIONS_H
#include "Config.h"

class Point {
public:
    Point();
    Point(const Point& p);
    Point(float* coords):x(coords[0]),y(coords[1]),z(coords[2]){};
    Point(float pX, float pY, float pZ):x(pX),y(pY),z(pZ){};
    
    void setNewPoint(float pX, float pY, float pZ);

    void null();

    bool isNull();

    float& operator[] (int idx) {
        switch(idx) {
            case 0: return x; break;
            case 1: return y; break;
            case 2: return z; break;
            default: return x; break;
        }
    }   
    float operator[] (int idx) const {
        switch(idx) {
            case 0: return x; break;
            case 1: return y; break;
            case 2: return z; break;
            default: return x; break;
        }
    }
    // returns the homogenous vector, i.e. a 4-dimensional vector with 1.0 as last dimension
    void getHomVector(float * pHom) const {
        pHom[0] =  x;
        pHom[1] =  y;
        pHom[2] =  z;
        pHom[3] =  1.0;
        // return pHom;
    }
    float x,y,z;
};

class Rotation: public Point {
    public:
		Rotation () : Point (0,0,0){};
		Rotation(float pX,float pY, float pZ): Point(pX,pY,pZ) {
			x = pX;
			y = pY;
			z = pZ;
		}

		Rotation(const Rotation& r) : Point(r) {
			x= r.x;
			y= r.y;
			z= r.z;
		};
};

class JointAngles {
public:
    JointAngles(){
        null();
    }

    JointAngles(const JointAngles& pJA) {
		for (int i = 0;i<NUM_SERVOS;++i)
			a[i] = pJA.a[i];
	}

    void setDefaultPosition() {
        a[0] = 0.0;
        a[1] = 0.0;
        a[2] = 0.0;
        a[3] = 0.0;
        a[4] = 0.0;
        a[5] = 0.0;
        a[6] = 0.0;
    }

    static JointAngles getDefaultPosition() { 
        JointAngles ja;
        ja.setDefaultPosition(); 
        return ja; 
    };

    float& operator[](int idx) {
		if ((idx >= 0) || ( idx < NUM_SERVOS))
			return a[idx];
        static float dummy(0);
		return dummy;
	}
	const float& operator[](int idx) const {
		if ((idx >= 0) || ( idx < NUM_SERVOS))
			return a[idx];
        static float dummy(0);
		return dummy;
	}

    void null() {
		for (int i = 0;i<NUM_SERVOS;++i)
			a[i] = 0.0;
	}

	bool isNull() {
		for (int i = 0;i<NUM_SERVOS;++i)
			if (a[i] != 0.0)
				return false;
		return true;
	}
private:
    float a[NUM_SERVOS];
};

class InputPosition {
public:
    InputPosition() {
        null();
    }

    InputPosition(const InputPosition& pIP): InputPosition() {
        position = pIP.position;
        orientation = pIP.orientation;
        gripperDistance = pIP.gripperDistance;
        angles = pIP.angles;
        tcpDeviation = pIP.tcpDeviation;
    }

    InputPosition(const Point& pPosition, const Rotation& pOrientation, const float pGripperDistance) {
        position = pPosition;
        orientation = pOrientation;
        gripperDistance = pGripperDistance;
        tcpDeviation.null();
        angles.null();
    };
    InputPosition(const Point& pPosition, const Rotation& pOrientation, const float pGripperDistance, const JointAngles& pAngles, const Point& pTcpDeviation) {
        position = pPosition;
        orientation = pOrientation;
        gripperDistance = pGripperDistance;
        angles = pAngles;
        tcpDeviation = pTcpDeviation;
    };

    void null() {
        orientation.null();
        position.null();
        gripperDistance = 0.0;
        angles.null();
        tcpDeviation.null();
    }

    bool isNull() {
        return position.isNull();
    }

private:
    Point position;
    Rotation orientation;
    float gripperDistance;
    Point tcpDeviation;
    JointAngles angles;
};

#endif