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

void Point::printContents() {
	Serial.print("X : ");
	Serial.print(x);
	Serial.print("\t");
	Serial.print("Y : ");
	Serial.print(y);
	Serial.print("\t");
	Serial.print("Z : ");
	Serial.print(z);
	Serial.print("\t");
}

void Rotation::printContents() {
	Serial.print("gamma : ");
	Serial.print(x);
	Serial.print("\t");
	Serial.print("beta : ");
	Serial.print(y);
	Serial.print("\t");
	Serial.print("alpha : ");
	Serial.print(z);
	Serial.print("\t");
}

void Point::setNewPoint(float pX, float pY,float pZ) {
	x = pX;
	y = pY;
	z = pZ;
}

void JointAngles::printContents() {
	Serial.print("Angles : [ ");
	Serial.print(a[0]);
	Serial.print("\t");
	Serial.print(a[1]);
	Serial.print("\t");
	Serial.print(a[2]);
	Serial.print("\t");
	Serial.print(a[3]);
	Serial.print("\t");
	Serial.print(a[4]);
	Serial.print("\t");
	Serial.print(a[5]);
	Serial.print("\t]");
}

void FullPosition::printContents() {
	Serial.print(" Position: ");
	position.printContents();
	Serial.print("\n Orientation: ");

	// Serial.print("gamma : ");
	Serial.print(orientation[0]*180/M_PI);
	Serial.print("\t");
	// Serial.print("beta : ");
	Serial.print(orientation[1]*180/M_PI);
	Serial.print("\t");
	// Serial.print("alpha : ");
	Serial.print(orientation[2]*180/M_PI);
	Serial.print("\t");
	
	//absolutely no clue why orientation print isn't working here
	// orientation.printContents();
	Serial.print("\n Joint Angles: ");
	angles.printContents();
}