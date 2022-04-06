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


};
