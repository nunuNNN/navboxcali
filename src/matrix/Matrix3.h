/**
 * @file Matrix3.h
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
#include <cmath>

#include "Vector3.h"
#include "Quaternion.h"

class Vect3;	class Quat;

#ifndef PI
#define PI		3.14159265358979
#endif

#ifndef EPS
#define EPS		(2.220446049e-16)
#endif


int		sign(double val, double eps=EPS);
double	range(double val, double minVal, double maxVal);
double	atan2Ex(double y, double x);

#define asinEx(x)		asin(range(x, -1.0, 1.0))
#define acosEx(x)		acos(range(x, -1.0, 1.0))
class Mat3 
{
public:
	double e00, e01, e02, e10, e11, e12, e20, e21, e22;

public:
	Mat3(void);
	Mat3(double xyz);
	Mat3(const double *pxyz);
	Mat3(double xx, double yy, double zz);
	Mat3(double xx, double xy, double xz,
		  double yx, double yy, double yz,
		  double zx, double zy, double zz );

	Mat3 operator*(const Mat3 &m) const;	// matrix multiplication
	Vect3 operator*(const Vect3 &v) const;	// vector cross multiplication

	friend Mat3 operator~(const Mat3 &m);	// matrix transposition
	friend Vect3 m2att(const Mat3 &Cnb);	// DCM to Euler angles 
};
