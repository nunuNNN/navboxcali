/**
 * @file Matrix3.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */ 

#include "Matrix3.h"

Mat3::Mat3(void)
{
}

Mat3::Mat3(double xyz)
{
	e00=e01=e02 =e10=e11=e12 =e20=e21=e22 =xyz;
}

Mat3::Mat3(const double *pxyz)
{
	e00=*pxyz++,e01=*pxyz++,e02=*pxyz++,
	e10=*pxyz++,e11=*pxyz++,e12=*pxyz++,
	e20=*pxyz++,e21=*pxyz++,e22=*pxyz  ;
}

Mat3::Mat3(double xx, double yy, double zz)
{
	e00=xx, e11=yy, e22=zz;
	e01=e02 =e10=e12 =e20=e21 =0.0;
}

Mat3::Mat3(double xx, double xy, double xz, 
		  double yx, double yy, double yz,
		  double zx, double zy, double zz )
{
	e00=xx,e01=xy,e02=xz; e10=yx,e11=yy,e12=yz; e20=zx,e21=zy,e22=zz;
}

Mat3 Mat3::operator*(const Mat3 &mat) const
{
	Mat3 mtmp;
	mtmp.e00 = e00*mat.e00 + e01*mat.e10 + e02*mat.e20;
	mtmp.e01 = e00*mat.e01 + e01*mat.e11 + e02*mat.e21;
	mtmp.e02 = e00*mat.e02 + e01*mat.e12 + e02*mat.e22;
	mtmp.e10 = e10*mat.e00 + e11*mat.e10 + e12*mat.e20;
	mtmp.e11 = e10*mat.e01 + e11*mat.e11 + e12*mat.e21;
	mtmp.e12 = e10*mat.e02 + e11*mat.e12 + e12*mat.e22;
	mtmp.e20 = e20*mat.e00 + e21*mat.e10 + e22*mat.e20;
	mtmp.e21 = e20*mat.e01 + e21*mat.e11 + e22*mat.e21;
	mtmp.e22 = e20*mat.e02 + e21*mat.e12 + e22*mat.e22;
	return mtmp;
}

Vect3 Mat3::operator*(const Vect3 &v) const
{
	return Vect3(e00*v.x+e01*v.y+e02*v.z,e10*v.x+e11*v.y+e12*v.z,e20*v.x+e21*v.y+e22*v.z);
}

Mat3 operator~(const Mat3 &m)
{
	return Mat3(m.e00,m.e10,m.e20, m.e01,m.e11,m.e21, m.e02,m.e12,m.e22);
}

Vect3 m2att(const Mat3 &Cnb)
{
	Vect3 att;
	att.x = asinEx(Cnb.e21);
	att.y = atan2Ex(-Cnb.e20, Cnb.e22);
	att.z = atan2Ex(-Cnb.e01, Cnb.e11);
	return att;
}



// determine the sign of 'val' with the sensitivity of 'eps'
int sign(double val, double eps)
{
	int s;

	if(val<-eps)
	{
		s = -1;
	}
	else if(val>eps)
	{
		s = 1;
	}
	else
	{
		s = 0; 
	}
	return s;
}

// set double value 'val' between range 'minVal' and 'maxVal'
double range(double val, double minVal, double maxVal)
{
	double res;

	if(val<minVal)
	{ 
		res = minVal; 
	}
	else if(val>maxVal)	
	{ 
		res = maxVal; 
	}
	else
	{ 
		res = val;
	}
	return res;
}

double atan2Ex(double y, double x)
{
	double res;

	if((sign(y)==0) && (sign(x)==0))
	{
		res = 0.0;
	}
	else
	{
		res = atan2(y, x);
	}
	return res;
}


