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

Vect3 Vect3::operator*(double f) const
{
	return Vect3(x*f, y*f, z*f);
}

Vect3& Vect3::operator*=(double f)
{ 
	x *= f, y *= f, z *= f;
	return *this;
}

Vect3 Vect3::operator*(const Vect3 &v) const
{
	return Vect3(this->y*v.z-this->z*v.y, this->z*v.x-this->x*v.z, this->x*v.y-this->y*v.x);
}

Vect3 Vect3::operator/(double f) const
{
	return Vect3(x/f, y/f, z/f);
}

Vect3 Vect3::operator+(const Vect3 &v) const
{
	return Vect3(this->x+v.x, this->y+v.y, this->z+v.z);
}

Vect3& Vect3::operator+=(const Vect3 &v)
{
	x += v.x, y += v.y, z += v.z;
	return *this;
}

Vect3 operator*(double f, const Vect3 &v)
{
	return Vect3(v.x*f, v.y*f, v.z*f);
}

