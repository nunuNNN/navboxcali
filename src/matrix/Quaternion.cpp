/**
 * @file Quaternion.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "CMath.hpp"


Quat::Quat(void)
{
}

Quat::Quat(double qq0, double qq1, double qq2, double qq3)
{
	q0=qq0, q1=qq1, q2=qq2, q3=qq3;
}

Quat::Quat(const double *pdata)
{
	q0=*pdata++, q1=*pdata++, q2=*pdata++, q3=*pdata++;
}
