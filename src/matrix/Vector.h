/**
 * @file Vector.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <iostream>
#include <cstring>

// Matrix Max Dimension define
#define MMD		34
#define MMD2	(MMD*MMD)

#ifndef INF
#define INF		(3.402823466e+30)
#endif

class Vect
{
public:
	int row, clm, rc;
	double dd[MMD];

	Vect(void);
	Vect(int row0, int clm0=1);
	Vect(int row0, double f);
	Vect(int row0, const double *pf);
};




