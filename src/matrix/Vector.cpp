/**
 * @file Vector.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Vector.h"


Vect::Vect(void)
{
}

Vect::Vect(int row0, int clm0)
{
	if(clm0==1) { row=row0; clm=1;   }
	else		{ row=1;    clm=clm0;}
	rc = row*clm;
}

Vect::Vect(int row0, double f)
{
	row=row0; clm=1; rc=row*clm;
	for(int i=0;i<row;i++) dd[i]=f;
}

Vect::Vect(int row0, const double *pf)
{
	row=row0; clm=1; rc=row*clm;
	memcpy(dd, pf, row*sizeof(double));
}


