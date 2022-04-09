/**
 * @file Matrix.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <iostream>
#include <cstring>

#include "Vector.h"


class Mat
{
public:
	int row, clm, rc;
	double dd[MMD2];

	Mat(void);
	Mat(int row0, int clm0);
	Mat(int row0, int clm0, double f);
	Mat(int row0, int clm0, const double *pf);
};