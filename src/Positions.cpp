#include "Positions.h"

Point::Point() {
    null();
}

Point::Point(const Point& p) {
	x = p.x;
	y = p.y;
	z = p.z;
}

void Point::null() {
	x = 0.0;
	y = 0.0;
	z = 0.0;
};

bool Point::isNull() {
    return((x==0)&&(y==0)&&(z==0));
}

void Point::setNewPoint(float pX, float pY,float pZ) {
	x = pX;
	y = pY;
	z = pZ;
}

