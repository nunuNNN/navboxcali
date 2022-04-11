/**
 * @file CMath.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <iostream>
#include <cmath>


#include "Matrix3.h"
#include "Vector3.h"
#include "Quaternion.h"

#include "Matrix.h"


// global para
#ifndef PI
#define PI		3.14159265358979
#endif
#ifndef EPS
#define EPS		(2.220446049e-16)
#endif

#define PI_2	(PI/2.0)
#define PI_4	(PI/4.0)
#define _2PI	(2.0*PI)

#define sqrt2	1.414213562373095	// sqrt(2) ...
#define sqrt3	1.732050807568877
#define sqrt5	2.236067977499790
#define sqrt6	2.449489742783178
#define sqrt7	2.645751311064591
#define sqrt8	2.828427124746190

#define DEG		(PI/180.0)		// arcdeg
#define MIN		(DEG/60.0)		// arcmin
#define SEC		(MIN/60.0)		// arcsec
#define HUR		3600.0			// hur
#define SHUR	60.0			// sqrt(hur)
#define DPH		(DEG/HUR)		// deg/h
#define DPSH	(DEG/SHUR)		// deg/sqrt(h)
#define G0		9.7803267714
#define MG		(G0/1.0e3)
#define UG		(G0/1.0e6)		// ug
#define UGPSHZ	(UG/1)			// ug/sqrt(Hz)
#define RE		6378137.0
#define PPM		1.0e-6

#define INFp5	(INF*0.5)

// constant define for short in KF P/Q/R setting
#define fXYZU(X,Y,Z,U)	1.0*(X)*(U),1.0*(Y)*(U),1.0*(Z)*(U)
#define fXXZU(X,Z,U)	fXYZU(X,X,Z,U)
#define fXYZ(X,Y,Z)		fXYZU(X,Y,Z,1.0)
#define fXXX(X)			fXYZ(X,X,X)
#define fXX6(X)			fXXX(X),fXXX(X)
#define fXX9(X)			fXX6(X),fXXX(X)
#define fOOO			fXXX(0.0)
#define fOO6			fXX6(0.0)
#define fOO9			fXX9(0.0)
#define fINF3			fXXX(INF)
#define fINF6			fXX6(INF)
#define fINF9			fXX9(INF)
#define fDEG3(X)		fXXX(X*DEG)

#define fXXZ(X,Z)		fXYZ(X,X,Z)
#define fdLLH(LL,H)		fXXZ((LL)/RE,(H))
#define fdPOS(LLH)		fdLLH(LLH,LLH)
#define fDPSH3(X)		fXXX(X*DPSH)

#define fDPH3(X)		fXXX(X*DPH)
#define fMG3(X)			fXXX(X*MG)
#define fUG3(X)			fXXX(X*UG)
#define fUGPSHZ3(X)		fXXX(X*UGPSHZ)
#define fPHI(EN,U)		fXXZU(EN,U,MIN)
#define fKPP(dpch,dk,dyaw)	fXYZ(dpch*DEG,dk,dyaw*DEG)

#define fdKG9(dkii,dkij)	(dkii)*PPM,(dkij)*SEC,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM,(dkij)*SEC,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM
#define fdKA6(dkii,dkij)	(dkii)*PPM,(dkij)*SEC,(dkij)*SEC,(dkii)*PPM,(dkij)*SEC,(dkii)*PPM



const Vect3 O31(0.0);
const Quat  qI(1.0,0.0,0.0,0.0);
const Mat3  I33(1,0,0, 0,1,0, 0,0,1), O33(0,0,0, 0,0,0, 0,0,0), One33(1.0);
