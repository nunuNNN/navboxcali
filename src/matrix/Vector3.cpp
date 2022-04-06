/**
 * @file Vector3.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "CMath.hpp"


Vect3::Vect3(void)
{
}

Vect3::Vect3(double xyz)
{
	x=y=z=xyz;
}

Vect3::Vect3(double xx, double yy, double zz)
{
	x=xx, y=yy, z=zz;
}

Vect3& Vect3::operator=(double f)
{
	x = y = z = f;
	return *this;
}

Vect3& Vect3::operator=(const double *pf)
{
	x = *pf++, y = *pf++, z = *pf;
	return *this;
}

