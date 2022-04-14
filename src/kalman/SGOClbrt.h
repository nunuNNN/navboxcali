/**
 * @file SGOClbrt.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include "typeReDefine.h"
#include "SGClbrt.h"

#pragma pack(4)
class SGOClbrt:public SGClbrt
{
public:
    double Kod, odmeast, odmeasT, dS0, distance;
	BOOL odVelOK;

    Vect3 IVno, IVni, Ifn, lvOD, vnOD, posOD, odZk;
    Mat3 Cbo, IMv, odMphi, odMvn, odMkappa, odMlever;		// Cbo: from body-frame to OD-frame
public:
	SGOClbrt(double ts);
	void Init(const SINS &sins0);
	virtual void SetFt(int nnq);
	virtual void SetHk(int nnq);
	virtual void Feedback(int nnq, double fbts);
    BOOL ODVelUpdate(double dS);
	int Update(const Vect3 *pwm, const Vect3 *pvm, double dS, int nSamples, double ts, int nSteps=5); 

};
#pragma pack()
