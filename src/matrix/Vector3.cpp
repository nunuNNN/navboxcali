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

#include <cmath>

#include "Vector3.h"

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

Vect3 Vect3::operator-(const Vect3 &v) const
{
	return Vect3(this->x-v.x, this->y-v.y, this->z-v.z);
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

Vect3& Vect3::operator-=(const Vect3 &v)
{ 
	x -= v.x, y -= v.y, z -= v.z;
	return *this;
}

Vect3 operator*(double f, const Vect3 &v)
{
	return Vect3(v.x*f, v.y*f, v.z*f);
}

Vect3 operator-(const Vect3 &v)
{
	return Vect3(-v.x, -v.y, -v.z);
}


Quat a2qua(double pitch, double roll, double yaw)
{
	pitch /= 2.0, roll /= 2.0, yaw /= 2.0;
    double	sp = sin(pitch), sr = sin(roll), sy = sin(yaw), 
			cp = cos(pitch), cr = cos(roll), cy = cos(yaw);
	Quat qnb;
    qnb.q0 = cp*cr*cy - sp*sr*sy;
    qnb.q1 = sp*cr*cy - cp*sr*sy;
    qnb.q2 = cp*sr*cy + sp*cr*sy;
    qnb.q3 = cp*cr*sy + sp*sr*cy;
	return qnb;
}

Quat a2qua(const Vect3 &att)
{
	return a2qua(att.x, att.y, att.z);
}


Quat rv2q(const Vect3 &rv)
{
#define F1	(   2 * 1)		// define: Fk=2^k*k! 
#define F2	(F1*2 * 2)
#define F3	(F2*2 * 3)
#define F4	(F3*2 * 4)
#define F5	(F4*2 * 5)
	double n2 = rv.x*rv.x+rv.y*rv.y+rv.z*rv.z, c, f;
	if(n2<(PI/180.0*PI/180.0))	// 0.017^2 
	{
		double n4=n2*n2;
		c = 1.0 - n2*(1.0/F2) + n4*(1.0/F4);
		f = 0.5 - n2*(1.0/F3) + n4*(1.0/F5);
	}
	else
	{
		double n_2 = sqrt(n2)/2.0;
		c = cos(n_2);
		f = sin(n_2)/n_2*0.5;
	}
	return Quat(c, f*rv.x, f*rv.y, f*rv.z);
}

Mat3 askew(const Vect3 &v)
{
	return Mat3(0,  -v.z, v.y, 
				 v.z, 0.0,  -v.x,
				-v.y, v.x, 0);
}

Mat3 a2mat(const Vect3 &att)
{
	double	si = sin(att.x), ci = cos(att.x),
			sj = sin(att.y), cj = cos(att.y),
			sk = sin(att.z), ck = cos(att.z);
	Mat3 Cnb;
	Cnb.e00 =  cj*ck - si*sj*sk;	Cnb.e01 =  -ci*sk;	Cnb.e02 = sj*ck + si*cj*sk;
	Cnb.e10 =  cj*sk + si*sj*ck;	Cnb.e11 =  ci*ck;	Cnb.e12 = sj*sk - si*cj*ck;
	Cnb.e20 = -ci*sj;				Cnb.e21 =  si;		Cnb.e22 = ci*cj;
	return Cnb;
}

double normInf(const Vect3 &v)
{
	double x = v.x>0 ? v.x : -v.x,
		   y = v.y>0 ? v.y : -v.y,
		   z = v.z>0 ? v.z : -v.z;
	if(x>y)	return x>z ? x : z;
	else    return y>z ? y : z;
}

double norm(const Vect3 &v)
{
	return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}

