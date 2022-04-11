/**
 * @file SGOCalbrt.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "SGOClbrt.h"

SGOClbrt::SGOClbrt(double ts):SGClbrt(26, 10, ts, 9)
{
    odmeast = 0.0;  odmeasT = 1.0;
    dS0 = 0.0;  distance = 0.0;
    lvOD = vnOD = O31;
    Kod = 1; Cbo = a2mat(Vect3(0.0, 0.0, 0.0));
    odVelOK = 0;

	// 0-14: phi,dvn,dpos,eb,db; 15-17: lvGNSS; 18-20: Kappa; 21-23: lvOD; 24: dtGNSS; 25: dyawGNSS
	// Hk(0:5,:) ...								// 0-5: SINS/GNSS-dvn,dpos
	Hk.SetMat3(6, 3, I33);							// 6-8: SINS/OD-dvn
	Hk(yawHkRow,2) = 1.0; Hk(yawHkRow,25) = -1.0; 	// 9: SINS/GNSS-dyaw
	SetMeasMask(1, 01777);
}

void SGOClbrt::Init(const SINS &sins0)
{
    SGClbrt::Init(sins0);
	sins.lever(lvOD, &posOD, &vnOD);
	odmeasT = 1.0;
	

	Pmax.Set2(fDEG3(10.0), fXXX(50.0), fdPOS(1.0e4),
		fDPH3(3600), fMG3(10.0), fXXX(1.1), fKPP(1,0.01,1), fXXX(10.0), 0.1, 1*DEG);
	Pmin.Set2(fPHI(0.1,0.6), fXXX(0.001), fdPOS(0.01),
		fDPH3(0.1), fUG3(100), fXXZ(0.01,0.01), fKPP(0.01,0.001,0.01), fXXX(0.001), 0.0001, 0.01*DEG);
	Pk.SetDiag2(fDEG3(1.0), fXXX(1.0), fdPOS(100.0),
		fDPH3(10), fMG3(1.0), fXXZ(1.1,1.1), fKPP(0.1,0.01,1.1), fXXX(1.0), 0.01, 1.1*DEG);
	Qt.Set2(fDPSH3(0.51), fUGPSHZ3(1000), fOO9, fOOO, fOOO, fOOO, 0.0, 0.0);
	Rt.Set2(fXXZ(0.2,0.6), fdLLH(10.0,30.0), fXXZ(0.1,0.1), 1.0*DEG);
	Rmax = Rt*100;  Rmin = Rt*0.0001;  Rb.SetBit(077, 0.5);
	FBTau = .10;
}

void SGOClbrt::SetFt(int nnq)
{
	SGClbrt::SetFt(18);
}

void SGOClbrt::SetHk(int nnq)
{
	SGClbrt::SetHk(18);
	Vect3 MV=sins.Mpv*sins.vn;
	Hk.SetClmVect3(0,24, -sins.an);
	Hk.SetClmVect3(3,24, -MV);
}

void SGOClbrt::Feedback(int nnq, double fbts)
{
	SGClbrt::Feedback(18, fbts);
	Cbo = Cbo*a2mat(Vect3(FBXk.dd[18],0.0,FBXk.dd[20]));
	Kod *= 1 - FBXk.dd[19];
	lvOD += *(Vect3*)&FBXk.dd[21];
	dtGNSSdelay += FBXk.dd[24];
	dyawGNSS += FBXk.dd[25];
}

int SGOClbrt::ODVelUpdate(double dS)
{
	if(dS<-1.0||dS>10.0) { dS = dS0; } dS0 = dS;  // if bad OD input 
	if(odmeast<EPS) {   // re-initialize
		IVno = IVni = Ifn = O31;
		IMv = odMlever = O33;
	}
	dS *= Kod;
	distance += dS;
	Vect3 dSn = sins.Cnb*(Vect3(Cbo.e01*dS,Cbo.e11*dS,Cbo.e21*dS)-sins.imu.phim*lvOD);  // dSn = Cnb*Cbo*dSb
	odmeast += sins.nts;
	if(odmeast>=odmeasT) {  // take measurement every (odmeasT)(s)
		odMphi = -askew(odmeast/2.0*Ifn+IVno);
		odMvn = odmeast*I33;
		odMkappa = Mat3( IMv.e02,-IMv.e01,-IMv.e00,
						  IMv.e12,-IMv.e11,-IMv.e10,
						  IMv.e22,-IMv.e21,-IMv.e20 );
		//odMlever = sins.Cnb*askew(-odmeast*sins.web);
		odZk = IVni - IVno; 
		odmeast = 0.0;
		return odVelOK=1;
	}
	else {
		IVno += dSn;  IVni += sins.vn*sins.nts; Ifn += sins.fn*sins.nts;
		IMv += dS*sins.Cnb;
		odMlever += sins.Cnb*askew(-sins.nts*sins.web);
		return odVelOK=0;
	}
}


int SGOClbrt::Update(const Vect3 *pwm, const Vect3 *pvm, double dS, int nn, double ts, int nSteps)
{
	int res = SGClbrt::Update(pwm, pvm, nn, ts, nSteps);
	sins.lever(lvOD, &posOD, &vnOD);
	ODVelUpdate(dS);

	if(odVelOK) {
		Hk.SetMat3(6, 0, odMphi); Hk.SetMat3(6, 3, odMvn); Hk.SetMat3(6, 18, odMkappa); Hk.SetMat3(6, 21, odMlever);  // lvOD
		*(Vect3*)&Zk.dd[6] = odZk;  // SINS/OD-dvn
		SetMeasFlag(0700);
	}
	return res;
}


