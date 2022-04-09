#pragma once

#include <iostream>
#include <cmath>


#include "Matrix3.h"
#include "Vector3.h"
#include "Quaternion.h"

#include "Vector.h"
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
#define RE		6378137.0

const Vect3 O31(0.0);
const Quat  qI(1.0,0.0,0.0,0.0);
const Mat3  I33(1,0,0, 0,1,0, 0,0,1), O33(0,0,0, 0,0,0, 0,0,0), One33(1.0);
