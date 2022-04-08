/**
 * @file SINS.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <cstdio>
#include "CMath.hpp"
#include "Earth.h"
#include "IMU.h"

class SINS
{
public:
    double ts, nts, tk;
	double mvnt, mvnT, velMax, hgtMin, hgtMax, afabar;
	int mvnk;
	CEarth eth;
	CIMU imu;
	CQuat qnb;
	CMat3 Cnb, Cnb0, Cbn, mvnCnb0, Kg, Ka;
	CVect3 wib, fb, fn, an, anbar, web, webbar, wnb, att, vn, mvn, mvni, mvnmax, mvnmin, vb, pos, eb, db, Ka2, tauGyro, tauAcc, _betaGyro, _betaAcc;
	CMat3 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp;	// for etm
	CVect3 lvr, vnL, posL; 
    CMat3 CW, MpvCnb;		// for lever arm
	BOOL isOpenloop;

public:
	CEarth eth;
	CIMU imu;

public:
    SINS(const Vect3 &att0, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);
    void Init(const Quat &qnb0=qI, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);    // initialization using quat attitude, velocity & position
	void Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts);		// SINS update using Gyro&Acc samples
    void lever(const CVect3 &dL=O31, CVect3 *ppos=NULL, CVect3* pvn=NULL);		// lever arm
    void etm(void);							// SINS error transform matrix coefficients

};


