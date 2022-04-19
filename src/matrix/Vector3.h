/**
 * @file Vector3.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <cstdio>

#include "Matrix3.h"
#include "Quaternion.h"

#pragma pack(4)
class Mat3;	class Quat;

class Vect3
{
public:
    double x, y, z;

public:
    Vect3(void);
    Vect3(double xyz);
    Vect3(double x, double y, double z);

    Vect3& operator=(double f);             // 每一个元素都等于一个数
    Vect3& operator=(const double *pf);     // 通过数组赋值向量
    Vect3 operator-(const Vect3 &v) const;	// vector subtraction
    Vect3 operator*(double f) const;        // vector multiply scale
    Vect3& operator*=(double f);			// vector multiply scale
    Vect3 operator*(const Vect3 &v) const;  // vector cross multiplication
    Vect3 operator/(double f) const;		// vector divide scale
    Vect3 operator+(const Vect3 &v) const;	// vector addition
    Vect3& operator+=(const Vect3 &v);	    // vector addition
    Vect3& operator-=(const Vect3 &v);	    // vector subtraction

    friend Vect3 operator*(double f, const Vect3 &v);   // scale multiply vector
    friend Vect3 operator-(const Vect3 &v);           // minus
	friend Quat a2qua(double pitch, double roll, double yaw);	// Euler angles to quaternion
	friend Quat a2qua(const Vect3 &att);        // Euler angles to quaternion
    friend Quat rv2q(const Vect3 &rv);          // rotation vector to quaternion
    friend Mat3 askew(const Vect3 &v);		    // askew matrix;
    friend Mat3 a2mat(const Vect3 &att);		// Euler angles to DCM 
    friend double normInf(const Vect3 &v);		// vector inf-norm
    friend double norm(const Vect3 &v);					// vector norm
};
#pragma pack()
