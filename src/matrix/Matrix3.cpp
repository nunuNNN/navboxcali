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

#include "CMath.hpp"

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
