/**
 * @file SGClbrt.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "CMath.hpp"
#include "SINS.h"
#include "Kalman.h"
#include "AVPInterp.h"
#pragma pack(4)
double  diffYaw(double yaw, double yaw0);


class SGClbrt:public Kalman
{
public:
	double meantdts, tdts, Pz0, innovation;
	int iter, ifn, adptOKi, measRes, maxStep;
	Mat Fk, Pk1;
	Vect Pxz, Qk, Kk, Hi;
	Vect3 meanfn;

	double posGNSSdelay, vnGNSSdelay, yawGNSSdelay, dtGNSSdelay, dyawGNSS, kfts;
	Vect3 lvGNSS;

public:
	SINS sins;
	AVPInterp avpi;

public:
    SGClbrt(void);
    SGClbrt(int nq0, int nr0, double ts);

    void Init(const SINS &sins0);

	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
	virtual void SetMeas(void) {};
	void SetMeasGNSS(const Vect3 &posgnss=O31, const double &vbgnss=0, double yawgnss=0.0, double qfactor=1.0);
	int TDUpdate(const Vect3 *pwm, const Vect3 *pvm, int nSamples, double ts, int nStep=1);  // Time-Distributed Update
	int Update(const Vect3 *pwm, const Vect3 *pvm, int nSamples, double ts, int nSteps=100); 

	BOOL IsZero(const Vect3 &v, double eps=EPS);		// psinsassert if all elements are zeros

};
#pragma pack()

