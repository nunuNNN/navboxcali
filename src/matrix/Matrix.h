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

#pragma once

#include <iostream>
#include <cstring>
#include <cstdarg>

#include "Vector3.h"

// Matrix Max Dimension define
#define MMD		34
#define MMD2	(MMD*MMD)

#ifndef INF
#define INF		(3.402823466e+30)
#endif

class Mat; class Vect;

class Vect
{
public:
	int row, clm, rc;
	double dd[MMD];

	Vect(void);
	Vect(int row0, int clm0=1);
	Vect(int row0, double f);
	Vect(int row0, const double *pf);

	void Set(double f, ...);
	void Set2(double f, ...);
	void SetBit(unsigned int bit, double f);// set element to f by bit mask

	Vect operator*(double f) const;			// vector multiply scale
	Vect& operator=(double f);				// every element equal to a same double
	Vect& operator+=(const Vect &v);		// vector addition
	Mat operator*(const Vect &v) const;		// 1xn vector multiply nx1 vector, or nx1 vector multiply 1xn vector
	double& operator()(int r);				// vector element
	friend Vect operator~(const Vect &v);	// vector transposition
};


class Mat
{
public:
	int row, clm, rc;
	double dd[MMD2];

	Mat(void);
	Mat(int row0, int clm0);
	Mat(int row0, int clm0, double f);
	Mat(int row0, int clm0, const double *pf);

	void SetDiag2(double f, ...);
	void SetClmVect3(int i, int j, const Vect3 &v);		// set i...(i+2)-row&j-column from CVect3
	void SetDiagVect3(int i, int j, const Vect3 &v);	// m(i,j)=v.i, m(i+1,j+1)=v.j, m(i+2,j+2)=v.k;
	void SetMat3(int i, int j, const Mat3 &m);			// set i...(i+2)-row&j...(j+2)-comumn from CMat3
	Vect GetRow(int i) const;							// get i-row from matrix
	double& operator()(int r, int c=-1);				// get element m(r,c)
	Mat& operator-=(const Mat &m0);						// matrix subtraction
	Mat operator*(double f) const;						// matrix multiply scale
	Vect operator*(const Vect &v) const;				// matrix multiply vector
	Mat& operator++();									// 1.0 + diagonal

	friend void RowMul(Mat &m, const Mat &m0, const Mat &m1, int r); // m(r,:)=m0(r,:)*m1
	friend void RowMulT(Mat &m, const Mat &m0, const Mat &m1, int r); // m(r,:)=m0(r,:)*m1'

};