/**
 * @file Earth.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Earth.h"

Earth::Earth(double a0, double f0, double g0)
{
	a = a0;	f = f0; wie = glv.wie; 
	b = (1-f)*a;
	e = sqrt(a*a-b*b)/a;	e2 = e*e;
	gn = O31;  pgn = 0;
	Update(O31);
}

void Earth::Update(const Vect3 &pos, const Vect3 &vn)
{
	this->pos = pos;  this->vn = vn;
	sl = sin(pos.x), cl = cos(pos.x), tl = sl/cl;
	double sq = 1-e2*sl*sl, sq2 = sqrt(sq);
	RMh = a*(1-e2)/sq/sq2+pos.z;	f_RMh = 1.0/RMh;
	RNh = a/sq2+pos.z;    clRNh = cl*RNh;  f_RNh = 1.0/RNh; f_clRNh = 1.0/clRNh;

    wnin = wnie = wnen = O31;
    sl2 = sl*sl;
    gn.z = -( glv.g0*(1+5.27094e-3*sl2)-3.086e-6*pos.z );
    gcc = pgn ? *pgn : gn;
}

