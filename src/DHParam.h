#ifndef DHPARAM_H
#define DHPARAM_H

#include <math.h>

class DHParam{
public:
	DHParam();
	DHParam(const float  pAlpha, const float pA, const float pD);
	void init(const float pAlpha, const float pA, const float pD);
	float getR() const { return _r; };
	const float getD() const { return _d; };
	const float getAlpha() const { return _alpha; };

	const float sinalpha() const { return sa; };
	const float cosalpha() const { return ca; };

private:
	float _r;
	float _d;
	float _alpha;
	float ca;
	float sa;
};

#endif