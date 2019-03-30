#ifndef DHPARAM_H
#define DHPARAM_H

#include <math.h>
// #include <algorithm>

#include "Config.h"
#include "Positions.h"
#include "Arduino.h"

class DHParam{
public:
	DHParam();
	DHParam(const float  pAlpha, const float pR, const float pD);
	void init(const float pAlpha, const float pR, const float pD);
	float getR() const { return _r; };
	const float getD() const { return _d; };
	const float getAlpha() const { return _alpha; };

	const float sinalpha() const { return sa; };
	const float cosalpha() const { return ca; };

	void printContent();

private:
	float _r;
	float _d;
	float _alpha;
	float ca;
	float sa;
};

class TransMatrix {
public:

    // friend std::ostream& operator<< (std::ostream &out, const TransMatrix &transMatrix);
	TransMatrix();

	TransMatrix(float pTheta, DHParam pDH);

	TransMatrix(FullPosition pFP);

	// TransMatrix(const TransMatrix& pTM);

	void init(float pTheta, DHParam pDH);

	void init(FullPosition pFP);

	//TODO: Need to fix
	TransMatrix& operator*= (const TransMatrix& rhs) {
		float tNew [NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS];
		for(int i=0; i<NUM_MATRIX_ROWS; ++i) {
			for(int j=0; j<NUM_MATRIX_COLUMNS; ++j) {
				for(int k=0; k<NUM_MATRIX_COLUMNS; ++k) {
					tNew[i*NUM_MATRIX_COLUMNS +j] += t[i*NUM_MATRIX_COLUMNS +k] * rhs[k*NUM_MATRIX_COLUMNS + j];
				}
			}
		}
		// std::copy(tNew, tNew+(NUM_MATRIX_COLUMNS*NUM_MATRIX_ROWS), t);
		for(int i=0; i<NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS; ++i) {
			t[i] = tNew[i];
		}
		// t = tNew;
		return *this;
	}

	TransMatrix operator* (const TransMatrix& rhs) {
		TransMatrix newTM = TransMatrix(0.0,DHParam(0,0,0));
		newTM.clear();
		// float tNew [NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS];
		for(int i=0; i<NUM_MATRIX_ROWS; ++i) {
			for(int j=0; j<NUM_MATRIX_COLUMNS; ++j) {
				for(int k=0; k<NUM_MATRIX_COLUMNS; ++k) {
					newTM.t[i*NUM_MATRIX_COLUMNS +j] += t[i*NUM_MATRIX_COLUMNS +k] * rhs[k*NUM_MATRIX_COLUMNS + j];
				}
			}
		}
		return newTM;
	}

	TransMatrix& operator= (const TransMatrix& rhs) {
		for(int i=0; i<NUM_MATRIX_ROWS * NUM_MATRIX_COLUMNS; ++i) {
			t[i] = rhs.t[i];
		}
		return *this;
	}

	TransMatrix multiply(int numRows, int numColumns, float* rhs);

	float& operator[](int idx) {
		if ((idx >= 0) || ( idx < NUM_MATRIX_COLUMNS*NUM_MATRIX_ROWS))
			return t[idx];
        static float dummy(0);
		return dummy;
	}
	const float& operator[](int idx) const {
		if ((idx >= 0) || ( idx < NUM_MATRIX_COLUMNS*NUM_MATRIX_ROWS))
			return t[idx];
        static float dummy(0);
		return dummy;
	}

	TransMatrix& inverse();

	TransMatrix& rotInverse();

	TransMatrix& transpose();

	void printContent();

	void dummy() {
		clear();
		t[0] = -1;
		t[3] = 190;
		t[5] = 1;
		t[6] = 1;
		t[9] = 1;
		t[11] = 220;
		t[15] = 1;
	}
	void clear();
	
	static void calcRotationMatrix(float x, float y, float z, TransMatrix& m);

	
private:
	void swapPos(float* pt, int a, int b);
	void calcGeneral();
	void calcHomogenous(FullPosition pos);
	float theta, alpha, r, d;
	float t [NUM_MATRIX_ROWS*NUM_MATRIX_COLUMNS];
};



#endif