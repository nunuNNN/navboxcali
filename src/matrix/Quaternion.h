/**
 * @file Quaternion.h
 * @author ld (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include <cstdio>

class Quat
{
public:
	double q0, q1, q2, q3;

	Quat(void);
	Quat(double qq0, double qq1=0.0, double qq2=0.0, double qq3=0.0);
	Quat(const double *pdata);

};
