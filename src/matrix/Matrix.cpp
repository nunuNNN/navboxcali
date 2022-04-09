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

void Vect::Set(double f, ...)
{
	// psinsassert(rc<=MMD);
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void Vect::Set2(double f, ...)
{
	// psinsassert(rc<=MMD);
	va_list vl;
	va_start(vl, f);
	for(int i=0; i<rc; i++)
	{ if(f>2*INF) break;  dd[i] = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void Vect::SetBit(unsigned int bit, double f)
{
	for(int i=0; i<rc; i++)		// assert(rc<32)
		if(bit&(0x01<<i)) dd[i]=f;
}

Vect Vect::operator*(double f) const
{
	Vect vtmp(row,clm);
	const double *p1=dd,*p1End=&dd[rc];
	for(double *p=vtmp.dd; p1<p1End; p++,p1++)  { *p=*p1*f; }
	return vtmp;
}

Vect& Vect::operator=(double f)
{
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++) { *p = f; }
	return *this;
}

Vect& Vect::operator+=(const Vect &v)
{
	// psinsassert(row==v.row&&clm==v.clm);
	const double *p1 = v.dd;
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p++,p1++)  { *p += *p1; }
	return *this;
}


Mat Vect::operator*(const Vect &v) const
{
	// psinsassert(clm==v.row);
	Mat mtmp(row,v.clm);
	if(row==1 && v.clm==1)  // (1x1) = (1xn)*(nx1)
	{
		double f = 0.0;
		for(int i=0; i<clm; i++)  f += dd[i]*v.dd[i];
		mtmp.dd[0] = f;
	}
	else    // (nxn) = (nx1)*(1xn)
	{
		double *p=mtmp.dd;
		for(const double *p1=&dd[0],*p1End=&dd[rc],*p2End=&v.dd[rc]; p1<p1End; p1++)
		{
			for(const double *p2=&v.dd[0]; p2<p2End; p2++)  {*p++ = *p1 * *p2;}
		}
	}
	return mtmp;
}

double& Vect::operator()(int r)
{
	return this->dd[r];
}

Vect operator~(const Vect &v)
{
	Vect vtmp=v;
	vtmp.row=v.clm; vtmp.clm=v.row;
	return vtmp;
}


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

void Mat::SetDiag2(double f, ...)
{
	*this = Mat(this->row, this->clm, 0.0);
	va_list vl;
	va_start(vl, f);
	double *p=dd, *pEnd=&dd[rc];
	for(int row1=row+1; p<pEnd; p+=row1)
	{ if(f>2*INF) break;  *p = f*f;  f = va_arg(vl, double);	}
	va_end(vl);
}

void Mat::SetClmVect3(int i, int j, const Vect3 &v)
{
	double *p=&dd[i*clm+j];
	*p = v.x; p += clm;
	*p = v.y; p += clm;
	*p = v.z;
}


void Mat::SetDiagVect3(int i, int j, const Vect3 &v)
{
	double *p=&dd[i*clm+j];
	*p = v.x;  p += clm+1;
	*p = v.y;  p += clm+1;
	*p = v.z;
}

void Mat::SetMat3(int i, int j, const Mat3 &m)
{
	double *p=&dd[i*clm+j];
	*(Vect3*)p = *(Vect3*)&m.e00;  p += clm;
	*(Vect3*)p = *(Vect3*)&m.e10;  p += clm;
	*(Vect3*)p = *(Vect3*)&m.e20;
}

Vect Mat::GetRow(int i) const
{
	Vect v(1, clm);
	const double *p1=&dd[i*clm], *pEnd=p1+clm;
	for(double *p=v.dd; p1<pEnd; p++,p1++) *p = *p1;
	return v;
}

double& Mat::operator()(int r, int c)
{
	if(c<0) c = r;
	return this->dd[r*this->clm+c];
}

Mat& Mat::operator-=(const Mat &m0)
{
	// psinsassert(row==m0.row&&clm==m0.clm);
	double *p=dd, *pEnd=&dd[rc]; const double *p1=m0.dd;
	while(p<pEnd)
	{ *p++ -= *p1++; } 
	return *this;
}

Mat Mat::operator*(double f) const
{
	Mat mtmp(row,clm);
	double *p=mtmp.dd, *pEnd=&mtmp.dd[rc]; const double *p1=this->dd;
	while(p<pEnd)
	{ *p++ = (*p1++) * f; } 
	return mtmp;
}

Vect Mat::operator*(const Vect &v) const
{
	// psinsassert(this->clm==v.row);
	Vect vtmp(this->row);
	double *p=vtmp.dd, *pEnd=&vtmp.dd[vtmp.row]; const double *p1ij=this->dd, *p2End=&v.dd[v.row];
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p2j=v.dd;
		for(; p2j<p2End; p1ij++,p2j++)	f += (*p1ij) * (*p2j);
		*p = f;
	}
	return vtmp;
}

Mat& Mat::operator++()
{
	int row1=row+1;
	for(double *p=dd, *pEnd=&dd[rc]; p<pEnd; p+=row1)	*p += 1.0;
	return *this;
}

void RowMul(Mat &m, const Mat &m0, const Mat &m1, int r)
{
	// psinsassert(m0.clm==m1.row);
	int rc0=r*m0.clm;
	double *p=&m.dd[rc0], *pEnd=p+m0.clm; const double *p0=&m0.dd[rc0], *p0End=p0+m0.clm, *p1j=m1.dd;
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p0j=p0, *p1jk=p1j++;
		for(; p0j<p0End; p0j++,p1jk+=m1.clm)	 f += (*p0j) * (*p1jk);
		*p = f;
	}
}

void RowMulT(Mat &m, const Mat &m0, const Mat &m1, int r)
{
	// psinsassert(m0.clm==m1.clm);
	int rc0=r*m0.clm;
	double *p=&m.dd[rc0], *pEnd=p+m0.clm; const double *p0=&m0.dd[rc0], *p0End=p0+m0.clm, *p1jk=m1.dd;
	for(; p<pEnd; p++)
	{
		double f=0.0; const double *p0j=p0;
		for(; p0j<p0End; p0j++,p1jk++)	 f += (*p0j) * (*p1jk);
		*p = f;
	}
}
