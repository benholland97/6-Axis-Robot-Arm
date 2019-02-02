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