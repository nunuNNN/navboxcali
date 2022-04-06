/**
 * @file Earth.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <cstdio>
#include "CMath.hpp"
#include "Glv.hpp"

const GLV   glv;

class Earth
{
public:
	double a, b;
	double f, e, e2;
	double wie;

	double sl, sl2, sl4, cl, tl, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;
	Vect3 pos, vn, wnie, wnen, wnin, gn, gcc, *pgn;
public:
	Earth(double a0=glv.Re, double f0=glv.f, double g0=glv.g0);
	void Update(const Vect3 &pos, const Vect3 &vn=O31);
};

