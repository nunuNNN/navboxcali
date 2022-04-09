/**
 * @file AVPInterp.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <cmath>
#include "CMath.hpp"
#include "typeReDefine.h"
#include "SINS.h"

class AVPInterp
{
#define AVPINUM 50
public:
	double ts;
	int ipush;
	Vect3 atti[AVPINUM], vni[AVPINUM], posi[AVPINUM];
	Vect3 att, vn, pos;
	AVPInterp(void);
	void Init(const SINS &sins, double ts, BOOL islever=0);
	void Init(const Vect3 &att0, const Vect3 &vn0, const Vect3 &pos0, double ts);
	void Push(const SINS &sins, BOOL islever=0);
	void Push(const Vect3 &attk, const Vect3 &vnk=O31, const Vect3 &posk=O31);
	int Interp(double tpast, int avp=0x7);	// AVP interpolation, where -AVPINUM*ts<=tpast<=0
};

