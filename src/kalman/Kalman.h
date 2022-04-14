/**
 * @file Kalman.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "CMath.hpp"

#pragma pack(4)
class Kalman
{
public:
    double kftk, zfdafa;
    int nq, nr;
    unsigned int kfcount, measflag, measflaglog, measmask;
    Mat Ft, Pk, Hk, Fading;
	Vect Xk, Zk, Qt, Rt, rts, RtTau, measstop, measlost, Xmax, Pmax, Pmin, Pset, Zfd, Zfd0, Zmax,
		Rmax, Rmin, Rbeta, Rb,						// measurement noise R adaptive
		FBTau, FBMax, FBOne, FBOne1, FBXk, FBTotal;	// feedback control
    int Rmaxcount[MMD], Rmaxcount0[MMD];

public:
	Kalman(void);
	Kalman(int nq0, int nr0);
	void Init(int nq0, int nr0);				// initialize Qk,Rk,P0...
	void SetRmaxcount(int cnt=5);
	virtual void SetFt(int nnq) = 0;			// process matrix setting
	virtual void SetHk(int nnq) = 0;			// measurement matrix setting
	virtual void SetMeas(void) = 0;				// set measurement
	virtual void Feedback(int nnq, double fbts);	// state feedback
	void RtFading(int i, double fdts);			// Rt growing if no measurment
	int RAdaptive(int i, double r, double Pr);	// Rt adaptive
	void SetMeasFlag(unsigned int flag);		// measurement flag setting
	void SetMeasMask(int type, unsigned int mask);
	void SetMeasStop(double stop=10.0, unsigned int meas=0xffffffff);
	void XPConstrain(void);						// Xk & Pk constrain: -Xmax<Xk<Xmax, Pmin<diag(Pk)<Pmax
	void PmaxPminCheck(void);
};
#pragma pack()


