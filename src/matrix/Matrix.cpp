/**
 * @file Matrix.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Matrix.h"

Mat::Mat(void)
{
}
	
Mat::Mat(int row0, int clm0)
{
	row=row0; clm=clm0; rc=row*clm;
}

Mat::Mat(int row0, int clm0, double f)
{
	row=row0; clm=clm0; rc=row*clm;
	for(double *pd=dd, *pEnd=&dd[rc]; pd<pEnd; pd++)  *pd = f;
}

Mat::Mat(int row0, int clm0, const double *pf)
{
	row=row0; clm=clm0; rc=row*clm;
	memcpy(dd, pf, rc*sizeof(double));
}

