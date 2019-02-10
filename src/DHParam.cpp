#include "DHParam.h"


DHParam::DHParam() {
	init(0,0,0);
};

DHParam::DHParam(const float  pAlpha, const float pR, const float pD) {
	init(pAlpha, pR, pD);
};

// initialize with the passed Denavit Hardenberg params and precompute sin/cos
void DHParam::init(const float pAlpha, const float pR, const float pD) {
		_r = pR;
		_d = pD;
		_alpha = pAlpha;

        ca = cos(_alpha);
        sa = sin(_alpha);
};


TransMatrix::TransMatrix(FullPosition pFP) {
	init(pFP);
}

TransMatrix::TransMatrix(float pTheta, DHParam pDH) {
	init(pTheta,pDH);
}

void TransMatrix::init(FullPosition pos) {
	clear();
	calcHomogenous(pos);
}

void TransMatrix::init(float pTheta, DHParam pDH) {
	theta = pTheta; 
	alpha = pDH.getAlpha(); 
	r = pDH.getR();
	d = pDH.getD();

	clear();
	calcGeneral();
}

void TransMatrix::calcGeneral() {
	// if((theta!=0) && (alpha!=0) && (r!=0) &&(d!=0)) {
		//row 0
		t[0*N+0] = cos(theta);
		t[0*N+1] = -sin(theta)*cos(alpha);
		t[0*N+2] = sin(theta)*sin(alpha);
		t[0*N+3] = r*cos(theta);
		//row 1
		t[1*N+0] = sin(theta);
		t[1*N+1] = cos(theta)*cos(alpha);
		t[1*N+2] = -cos(theta)*sin(alpha);
		t[1*N+3] = r*sin(theta);
		//row 3
		t[2*N+0] = 0;
		t[2*N+1] = sin(alpha);
		t[2*N+2] = cos(alpha);
		t[2*N+3] = d;
		//row 4
		t[3*N+0] = 0;
		t[3*N+1] = 0;
		t[3*N+2] = 0;
		t[3*N+3] = 1;

	// }
}

void TransMatrix::calcHomogenous(FullPosition pos) {
	//Extract roll, pitch, yaw trig calculations
	float sX = sin(pos.orientation[0]);
    float cX = cos(pos.orientation[0]);
    float sY = sin(pos.orientation[1]);
    float cY = cos(pos.orientation[1]);    
    float sZ= sin(pos.orientation[2]);
    float cZ = cos(pos.orientation[2]);
	//row 0
	t[0*N+0] = cZ*cY;
	t[0*N+1] = cZ*sY*sX - sZ*cX;
	t[0*N+2] = cZ*sY*cX + sZ*sX;
	t[0*N+3] = pos.position[0];
	//row 1
	t[1*N+0] = sZ*cY;
	t[1*N+1] = sZ*sY*sX + cZ*cZ;
	t[1*N+2] = sZ*sY*cX - cZ*sX;
	t[1*N+3] = pos.position[1];
	//row 2
	t[2*N+0] = -sY;
	t[2*N+1] = cY*sX;
	t[2*N+2] = cY*cX;
	t[2*N+3] = pos.position[2];
	//row 3
	t[3*N+0] = 0;
	t[3*N+1] = 0;
	t[3*N+2] = 0;
	t[3*N+3] = 1;

	// //row 0
	// t[0*N+0] = cX*cY*cZ - sX*sZ;
	// t[0*N+1] = -cX*cY*sZ - sX*cZ;
	// t[0*N+2] = cX*sY;
	// t[0*N+3] = pos.position[0];
	// //row 1
	// t[1*N+0] = sX*cY*cZ + cX*sZ;
	// t[1*N+1] = -sX*cY*sZ + cX*cZ;
	// t[1*N+2] = sX*sY;
	// t[1*N+3] = pos.position[1];
	// //row 2
	// t[2*N+0] = -sY*cZ;
	// t[2*N+1] = sY*sZ;
	// t[2*N+2] = cY;
	// t[2*N+3] = pos.position[2];
	// //row 3
	// t[3*N+0] = 0;
	// t[3*N+1] = 0;
	// t[3*N+2] = 0;
	// t[3*N+3] = 1;
}




TransMatrix TransMatrix::multiply(int numRows, int numColumns, float* rhs) {
	if(numRows != N) return *this;
	TransMatrix newTM = TransMatrix(0.0,DHParam(0,0,0));
	newTM.clear();
	for(int i=0; i<NUM_MATRIX_ROWS; ++i) {
		for(int j=0; j<numColumns; ++j) {
			for(int k=0; k<N; ++k) {
				newTM.t[i*numColumns+j] += t[i*N+k] * rhs[k*numColumns+j];
			}
		}
	}
	return newTM;
}


//ripped - may be wrong
TransMatrix& TransMatrix::inverse() {
	float tNew [NUM_MATRIX_ROWS*N];
	//row 0
	tNew[0*N + 0] = t[0*N + 0];
	tNew[0*N + 1] = t[1*N + 0];
	tNew[0*N + 2] = t[2*N + 0];
	tNew[0*N + 3] = -t[0*N + 0]*t[0*N + 3]-t[1*N + 0]*t[1*N + 3]-t[2*N + 0]*t[2*N + 3];
	//row 1
	tNew[1*N + 0] = t[0*N + 1];
	tNew[1*N + 1] = t[1*N + 1];
	tNew[1*N + 2] = t[2*N + 1];
	tNew[1*N + 3] = -t[0*N + 1]*t[0*N + 3]-t[1*N + 1]*t[1*N + 3]-t[2*N + 1]*t[2*N + 3];
	//row 2
	tNew[2*N + 0] = t[0*N + 2];
	tNew[2*N + 1] = t[1*N + 2];
	tNew[2*N + 2] = t[2*N + 2];
	tNew[2*N + 3] = -t[0*N + 2]*t[0*N + 3]-t[1*N + 2]*t[1*N + 3]-t[2*N + 2]*t[2*N + 3];
	//row 3
	tNew[3*N + 0] = 0.0;
	tNew[3*N + 1] = 0.0;
	tNew[3*N + 2] = 0.0;
	tNew[3*N + 3] = 1.0;
		
	std::copy(tNew, tNew+(N*NUM_MATRIX_ROWS), t);
	
	return *this;
}

void TransMatrix::printContent() {
	Serial.print("\n");
	for(int i=0; i<16; i+=4) {
		for(int j=0; j<4; ++j) {
			Serial.print(t[i+j]);
			Serial.print("\t");
		}
		Serial.print("\n");
	}
	Serial.print("\n");

}

void TransMatrix::clear() {
	for(int i=0; i<NUM_MATRIX_ROWS*N; ++i) {
		t[i] = 0;
	}
}

void TransMatrix::calcRotationMatrix(float x, float y, float z, TransMatrix& m) {
	float sX = sin(x);
    float cX = cos(x);
    float sY = sin(y);
    float cY = cos(y);    
    float sZ= sin(z);
    float cZ = cos(z);
	//row 0
	m.t[0*N+0] = cZ*cY;
	m.t[0*N+1] = cZ*sY*sX - sZ*cX;
	m.t[0*N+2] = cZ*sY*cX + sZ*sX;
	m.t[0*N+3] = 0;
	//row 1
	m.t[1*N+0] = sZ*cY;
	m.t[1*N+1] = sZ*sY*sX + cZ*cZ;
	m.t[1*N+2] = sZ*sY*cX - cZ*sX;
	m.t[1*N+3] = 0;
	//row 2
	m.t[2*N+0] = -sY;
	m.t[2*N+1] = cY*sX;
	m.t[2*N+2] = cY*cX;
	m.t[2*N+3] = 0;
	//row 3
	m.t[3*N+0] = 0;
	m.t[3*N+1] = 0;
	m.t[3*N+2] = 0;
	m.t[3*N+3] = 1;
}

// std::ostream& operator<< (std::ostream &out, const TransMatrix &tm) {
//     out <<"\n" << tm.t[0] << "\t" << tm.t[1] << "\t" << tm.t[2] << "\t" << tm.t[3] << "\n" 
// 		<<"\n" << tm.t[4] << "\t" << tm.t[5] << "\t" << tm.t[6] << "\t" << tm.t[7] << "\n" 
// 		<<"\n" << tm.t[8] << "\t" << tm.t[9] << "\t" << tm.t[10] << "\t" << tm.t[11] << "\n" 
// 		<<"\n" << tm.t[12] << "\t" << tm.t[13] << "\t" << tm.t[14] << "\t" << tm.t[15] << "\n";

//     return out; // return std::ostream so we can chain calls to operator<<
// }
 