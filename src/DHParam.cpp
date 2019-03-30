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

void DHParam::printContent() {
	Serial.print("Alpha: ");
	Serial.print(_alpha);
	Serial.print("\t");
	Serial.print("R: ");
	Serial.print(_r);
	Serial.print("\t");
	Serial.print("D: ");
	Serial.print(_d);
	Serial.print("\n");
}

TransMatrix::TransMatrix() {
	init(0,DHParam());
	clear();
}


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
	float sA = sin(pos.orientation[0]);
    float cA = cos(pos.orientation[0]);
    float sB = sin(pos.orientation[1]);
    float cB = cos(pos.orientation[1]);    
    float sG= sin(pos.orientation[2]);
    float cG = cos(pos.orientation[2]);

	// //x' -> y' -> z'
	// //row 0
	// t[0*N+0] = cB*cG;
	// t[0*N+1] = -cB*sG;
	// t[0*N+2] = sB;
	// t[0*N+3] = 0;
	// //row 1
	// t[1*N+0] = sA*sG + sA*sB*cG;
	// t[1*N+1] = cA*cG - sA*sB*sG;
	// t[1*N+2] = -sA*cB;
	// t[1*N+3] = 0;
	// //row 2
	// t[2*N+0] = -sA*sG - cA*sB*cG;
	// t[2*N+1] = sA*cG + cA*sB*sG;
	// t[2*N+2] = cA*cB;
	// t[2*N+3] = 0;
	// //row 3
	// t[3*N+0] = 0;
	// t[3*N+1] = 0;
	// t[3*N+2] = 0;
	// t[3*N+3] = 1;

	//walter
	//row 0
	t[0*N+0] = cG*cB;
	t[0*N+1] = cG*sB*sA - sG*cA;
	t[0*N+2] = cG*sB*cA + sG*sA;
	t[0*N+3] = pos.position[0];
	//row 1
	t[1*N+0] = sG*cB;
	t[1*N+1] = sG*sB*sA + cG*cG;
	t[1*N+2] = sG*sB*cA - cG*sA;
	t[1*N+3] = pos.position[1];
	//row 2
	t[2*N+0] = -sB;
	t[2*N+1] = cB*sA;
	t[2*N+2] = cB*cA;
	t[2*N+3] = pos.position[2];
	//row 3
	t[3*N+0] = 0;
	t[3*N+1] = 0;
	t[3*N+2] = 0;
	t[3*N+3] = 1;

	// //row 0
	// t[0*N+0] = cA*cB*cG - sA*sG;
	// t[0*N+1] = -cA*cB*sG - sA*cG;
	// t[0*N+2] = cA*sB;
	// t[0*N+3] = pos.position[0];
	// //row 1
	// t[1*N+0] = sA*cB*cG + cA*sG;
	// t[1*N+1] = -sA*cB*sG + cA*cG;
	// t[1*N+2] = sA*sB;
	// t[1*N+3] = pos.position[1];
	// //row 2
	// t[2*N+0] = -sB*cG;
	// t[2*N+1] = sB*sG;
	// t[2*N+2] = cB;
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

TransMatrix& TransMatrix::rotInverse() {
	float tNew[NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS];
	float det = t[0*N + 0] * (t[1*N + 1] * t[2*N + 2] - t[2*N + 1] * t[1*N + 2]) -
             	t[0*N + 1] * (t[1*N + 0] * t[2*N + 2] - t[1*N + 2] * t[2*N + 0]) +
            	t[0*N + 2] * (t[1*N + 0] * t[2*N + 1] - t[1*N + 1] * t[2*N + 0]);
	double invdet = 1 / det;

	tNew[0*N + 0] = (t[1*N + 1] * t[2*N + 2] - t[2*N + 1] * t[1*N + 2]) * invdet;
	tNew[0*N + 1] = (t[0*N + 2] * t[2*N + 1] - t[0*N + 1] * t[2*N + 2]) * invdet;
	tNew[0*N + 2] = (t[0*N + 1] * t[1*N + 2] - t[0*N + 2] * t[1*N + 1]) * invdet;
	tNew[0*N + 3] = 0.0;

	tNew[1*N + 0] = (t[1*N + 2] * t[2*N + 0] - t[1*N + 0] * t[2*N + 2]) * invdet;
	tNew[1*N + 1] = (t[0*N + 0] * t[2*N + 2] - t[0*N + 2] * t[2*N + 0]) * invdet;
	tNew[1*N + 2] = (t[1*N + 0] * t[0*N + 2] - t[0*N + 0] * t[1*N + 2]) * invdet;
	tNew[1*N + 3] = 0.0;

	tNew[2*N + 0] = (t[1*N + 0] * t[2*N + 1] - t[2*N + 0] * t[1*N + 1]) * invdet;
	tNew[2*N + 1] = (t[2*N + 0] * t[0*N + 1] - t[0*N + 0] * t[2*N + 1]) * invdet;
	tNew[2*N + 2] = (t[0*N + 0] * t[1*N + 1] - t[1*N + 0] * t[0*N + 1]) * invdet;
	tNew[2*N + 3] = 0.0;

	tNew[3*N + 0] = 0.0;
	tNew[3*N + 1] = 0.0;
	tNew[3*N + 2] = 0.0;
	tNew[3*N + 3] = 0.0;

	for(int i=0; i<NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS; ++i) {
		t[i] = tNew[i];
	}
		
	return *this;
}

TransMatrix& TransMatrix::transpose() {
	swapPos(t,0*N+1,1*N+0);
	swapPos(t,0*N+2,2*N+0);
	swapPos(t,1*N+2,2*N+1);
	
	t[0*N + 3] = 0.0;
	t[1*N + 3] = 0.0;
	t[2*N + 3] = 0.0;
	t[3*N + 0] = 0.0;
	t[3*N + 1] = 0.0;
	t[3*N + 2] = 0.0;
	t[3*N + 3] = 0.0;
	
	return *this;

}

void TransMatrix::swapPos(float* pT, int a, int b) {
	float temp = pT[a];
	pT[a] = pT[b];
	pT[b] = temp;
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

	for(int i=0; i<NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS; ++i) {
		t[i] = tNew[i];
	}
		
	// std::copy(tNew, tNew+(N*NUM_MATRIX_ROWS), t);
	
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

void TransMatrix::calcRotationMatrix(float a, float b, float g, TransMatrix& m) {
	float sA = sin(a);
    float cA = cos(a);
    float sB = sin(b);
    float cB = cos(b);    
    float sG= sin(g);
    float cG = cos(g);
	// //x' -> y' -> z'
	// //row 0
	// m.t[0*N+0] = cB*cG;
	// m.t[0*N+1] = -cB*sG;
	// m.t[0*N+2] = sB;
	// m.t[0*N+3] = 0;
	// //row 1
	// m.t[1*N+0] = sA*sG + sA*sB*cG;
	// m.t[1*N+1] = cA*cG - sA*sB*sG;
	// m.t[1*N+2] = -sA*cB;
	// m.t[1*N+3] = 0;
	// //row 2
	// m.t[2*N+0] = -sA*sG - cA*sB*cG;
	// m.t[2*N+1] = sA*cG + cA*sB*sG;
	// m.t[2*N+2] = cA*cB;
	// m.t[2*N+3] = 0;
	// //row 3
	// m.t[3*N+0] = 0;
	// m.t[3*N+1] = 0;
	// m.t[3*N+2] = 0;
	// m.t[3*N+3] = 1;

	//walter
	//row 0
	m.t[0*N+0] = cG*cB;
	m.t[0*N+1] = cG*sB*sA - sG*cA;
	m.t[0*N+2] = cG*sB*cA + sG*sA;
	m.t[0*N+3] = 0;
	//row 1
	m.t[1*N+0] = sG*cB;
	m.t[1*N+1] = sG*sB*sA + cG*cG;
	m.t[1*N+2] = sG*sB*cA - cG*sA;
	m.t[1*N+3] = 0;
	//row 2
	m.t[2*N+0] = -sB;
	m.t[2*N+1] = cB*sA;
	m.t[2*N+2] = cB*cA;
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
 