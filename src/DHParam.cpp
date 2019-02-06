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
	for(int i=0; i<NUM_MATRIX_ROWS; ++i) {
		for(int j=0; j<NUM_MATRIX_COLUMNS; ++j) {
			t[i*NUM_MATRIX_COLUMNS+j] = 0;
		}
	}	
	calcHomogenous(pos);
}

void TransMatrix::init(float pTheta, DHParam pDH) {
	a= pTheta; 
	b = pDH.getAlpha(); 
	c = pDH.getR();
	d = pDH.getD();

	for(int i=0; i<NUM_MATRIX_ROWS; ++i) {
		for(int j=0; j<NUM_MATRIX_COLUMNS; ++j) {
			t[i*NUM_MATRIX_COLUMNS+j] = 0;
		}
	}	
	calcGeneral();
}

void TransMatrix::calcGeneral() {
	if((a!=0) && (b!=0) && (c!=0) &&(d!=0)) {
		//row 0
		t[0*NUM_MATRIX_COLUMNS+0] = cos(a);
		t[0*NUM_MATRIX_COLUMNS+1] = -sin(a)*cos(b);
		t[0*NUM_MATRIX_COLUMNS+2] = sin(a)*sin(b);
		t[0*NUM_MATRIX_COLUMNS+3] = c*cos(a);
		//row 1
		t[1*NUM_MATRIX_COLUMNS+0] = sin(a);
		t[1*NUM_MATRIX_COLUMNS+1] = cos(a)*cos(b);
		t[1*NUM_MATRIX_COLUMNS+2] = -cos(a)*sin(b);
		t[1*NUM_MATRIX_COLUMNS+3] = c*sin(a);
		//row 3
		t[2*NUM_MATRIX_COLUMNS+0] = 0;
		t[2*NUM_MATRIX_COLUMNS+1] = sin(b);
		t[2*NUM_MATRIX_COLUMNS+2] = cos(b);
		t[2*NUM_MATRIX_COLUMNS+3] = d;
		//row 4
		t[3*NUM_MATRIX_COLUMNS+0] = 0;
		t[3*NUM_MATRIX_COLUMNS+1] = 0;
		t[3*NUM_MATRIX_COLUMNS+2] = 0;
		t[3*NUM_MATRIX_COLUMNS+3] = 1;
	}
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
	t[0*NUM_MATRIX_COLUMNS+0] = cZ*cY;
	t[0*NUM_MATRIX_COLUMNS+1] = cZ*sY*sX - sZ*cX;
	t[0*NUM_MATRIX_COLUMNS+2] = cZ*sY*cX + sZ*sX;
	t[0*NUM_MATRIX_COLUMNS+3] = pos.position[0];
	//row 1
	t[1*NUM_MATRIX_COLUMNS+0] = sZ*cY;
	t[1*NUM_MATRIX_COLUMNS+1] = sZ*sY*sX + cZ*cZ;
	t[1*NUM_MATRIX_COLUMNS+2] = sZ*sY*cX - cZ*sX;
	t[1*NUM_MATRIX_COLUMNS+3] = pos.position[1];
	//row 2
	t[2*NUM_MATRIX_COLUMNS+0] = -sY;
	t[2*NUM_MATRIX_COLUMNS+1] = cY*sX;
	t[2*NUM_MATRIX_COLUMNS+2] = cY*cX;
	t[2*NUM_MATRIX_COLUMNS+3] = pos.position[2];
	//row 3
	t[3*NUM_MATRIX_COLUMNS+0] = 0;
	t[3*NUM_MATRIX_COLUMNS+1] = 0;
	t[3*NUM_MATRIX_COLUMNS+2] = 0;
	t[3*NUM_MATRIX_COLUMNS+3] = 1;

}

TransMatrix& TransMatrix::multiply(int numRows, int numColumns, float* rhs) {
	if(numRows != NUM_MATRIX_COLUMNS) return *this;
	float tNew [NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS];
	for(int i=0; i<NUM_MATRIX_ROWS; ++i) {
		for(int j=0; j<NUM_MATRIX_COLUMNS; ++j) {
			for(int k=0; k<NUM_MATRIX_COLUMNS; ++k) {
				tNew[i*NUM_MATRIX_COLUMNS +j] += t[i*NUM_MATRIX_COLUMNS +k] * rhs[k*NUM_MATRIX_COLUMNS + j];
			}
		}
	}
	std::copy(tNew, tNew+(NUM_MATRIX_COLUMNS*NUM_MATRIX_ROWS), t);
	return *this;
}


//ripped - may be wrong
TransMatrix& TransMatrix::inverse() {
	float tNew [NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS];
	//row 0
	tNew[0*NUM_MATRIX_COLUMNS + 0] = t[0*NUM_MATRIX_COLUMNS + 0];
	tNew[0*NUM_MATRIX_COLUMNS + 1] = t[1*NUM_MATRIX_COLUMNS + 0];
	tNew[0*NUM_MATRIX_COLUMNS + 2] = t[2*NUM_MATRIX_COLUMNS + 0];
	tNew[0*NUM_MATRIX_COLUMNS + 3] = -t[0*NUM_MATRIX_COLUMNS + 0]*t[0*NUM_MATRIX_COLUMNS + 3]-t[1*NUM_MATRIX_COLUMNS + 0]*t[1*NUM_MATRIX_COLUMNS + 3]-t[2*NUM_MATRIX_COLUMNS + 0]*t[2*NUM_MATRIX_COLUMNS + 3];
	//row 1
	tNew[1*NUM_MATRIX_COLUMNS + 0] = t[0*NUM_MATRIX_COLUMNS + 1];
	tNew[1*NUM_MATRIX_COLUMNS + 1] = t[1*NUM_MATRIX_COLUMNS + 1];
	tNew[1*NUM_MATRIX_COLUMNS + 2] = t[2*NUM_MATRIX_COLUMNS + 1];
	tNew[1*NUM_MATRIX_COLUMNS + 3] = -t[0*NUM_MATRIX_COLUMNS + 1]*t[0*NUM_MATRIX_COLUMNS + 3]-t[1*NUM_MATRIX_COLUMNS + 1]*t[1*NUM_MATRIX_COLUMNS + 3]-t[2*NUM_MATRIX_COLUMNS + 1]*t[2*NUM_MATRIX_COLUMNS + 3];
	//row 2
	tNew[2*NUM_MATRIX_COLUMNS + 0] = t[0*NUM_MATRIX_COLUMNS + 2];
	tNew[2*NUM_MATRIX_COLUMNS + 1] = t[1*NUM_MATRIX_COLUMNS + 2];
	tNew[2*NUM_MATRIX_COLUMNS + 2] = t[2*NUM_MATRIX_COLUMNS + 2];
	tNew[2*NUM_MATRIX_COLUMNS + 3] = -t[0*NUM_MATRIX_COLUMNS + 2]*t[0*NUM_MATRIX_COLUMNS + 3]-t[1*NUM_MATRIX_COLUMNS + 2]*t[1*NUM_MATRIX_COLUMNS + 3]-t[2*NUM_MATRIX_COLUMNS + 2]*t[2*NUM_MATRIX_COLUMNS + 3];
	//row 3
	tNew[3*NUM_MATRIX_COLUMNS + 0] = 0.0;
	tNew[3*NUM_MATRIX_COLUMNS + 1] = 0.0;
	tNew[3*NUM_MATRIX_COLUMNS + 2] = 0.0;
	tNew[3*NUM_MATRIX_COLUMNS + 3] = 1.0;
		
	std::copy(tNew, tNew+(NUM_MATRIX_COLUMNS*NUM_MATRIX_ROWS), t);
	
	return *this;
}