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

#include <iostream>

#include "CMath.hpp"
#include "Earth.h"
#include "IMU.h"
#include "typeReDefine.h"

#pragma pack(4)
extern const Vect3	O31;
extern const Quat  qI;

class SINS
{
public:
    double ts, nts, tk;
	double velMax, hgtMin, hgtMax, afabar;

	Quat qnb;
	Mat3 Cnb, Cbn, Kg, Ka;
	Vect3 eb, db, wib, web, wnb, fb, fn, an, anbar, webbar; 
	Vect3 att, vn, vb, pos, Ka2, _betaGyro, _betaAcc;
	Mat3 Maa, Mav, Map, Mva, Mvv, Mvp, Mpv, Mpp;	// for etm
	Vect3 lvr, vnL, posL; 
    Mat3 CW, MpvCnb;		// for lever arm
	BOOL isOpenloop, isOutlever;

public:
	Earth eth;
	IMU imu;

public:
    SINS(const Vect3 &att0, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);
	SINS(const Quat &qnb0=qI, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);
    void Init(const Quat &qnb0=qI, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);    // initialization using quat attitude, velocity & position
	void Update(const Vect3 *pwm, const Vect3 *pvm, int nSamples, double ts);		// SINS update using Gyro&Acc samples
    void lever(const Vect3 &dL=O31, Vect3 *ppos=NULL, Vect3* pvn=NULL);		// lever arm
    void etm(void);							// SINS error transform matrix coefficients

};
#pragma pack()


