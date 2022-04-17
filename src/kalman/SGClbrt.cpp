/**
 * @file SGClbrt.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "SGClbrt.h"

double diffYaw(double yaw, double yaw0)
{
	double dyaw = yaw-yaw0;
	if(dyaw>=PI) dyaw-=_2PI;
	else if(dyaw<=-PI) dyaw+=_2PI;
	return dyaw;
}


SGClbrt::SGClbrt(void)
{
}

SGClbrt::SGClbrt(int nq0, int nr0, double ts)
{
    Kalman::Init(nq0, nr0);
	posGNSSdelay = vnGNSSdelay = yawGNSSdelay = dtGNSSdelay = dyawGNSS = -0.0;
	kfts = ts;
	lvGNSS = O31;
	// 0-14: phi,dvn,dpos,eb,db; 15-17: lvGNSS; 18: dtGNSS;
	Hk(0,3) = Hk(1,4) = Hk(2,5) = 1.0;		// dvn
	Hk(3,6) = Hk(4,7) = Hk(5,8) = 1.0;		// dpos
	Hk(6,2) = 1.0;							// dyaw
	SetMeasMask(1, 077);
	SetMeasStop(1.0);
}

void SGClbrt::Init(const SINS &sins0)
{
	sins = sins0;  kftk = sins.tk;
    Fk = Pk1 = Mat(nq,nq, 0.0);
    Pxz = Qk = Kk = Vect(nr, 0.0);
    meantdts = 1.0; tdts = 0.0; // 分片kalman的预测时间间隔。建议10ms
	iter = -2;  ifn = 0;
    maxStep = 2*(nq+nr)+3;
	meanfn = O31;
    SetMeasFlag(0);

	sins.lever(-lvGNSS);  sins.pos = sins.posL;    // sins0.pos is GNSS pos
	avpi.Init(sins, kfts, 1);

	{  // MEMS grade
		Pmax.Set2(fDEG3(50.0),  fXXX(100.0),  fdPOS(1.0e6), fDPH3(5000.0),  fMG3(30.0),
			fXXX(100.0), 0.5, fdKG9(1000.0,900.0), fdKA6(100.0,100.0));
		Pmin.Set2(fPHI(1,1),  fXXX(0.0001),  fdPOS(.001),  fDPH3(0.001),  fUG3(1.0),
			fXXX(0.001), 0.0001, fdKG9(1.0,1.0), fdKA6(1.0,1.0));
		Pk.SetDiag2(fPHI(600,600),  fXXX(1.0),  fdPOS(100.0),  fDPH3(1000.0),  fMG3(10.0),
			fXXX(1.0),  0.01,  fdKG9(1000.0,90.0), fdKA6(10.0,10.0));
		Qt.Set2(fDPSH3(1.1),  fUGPSHZ3(10.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
		Xmax.Set(fINF9,  fDPH3(3600.0),  fMG3(50.0),
			fXXX(10.0), 0.5,  fdKG9(1000.0,900.0),  fdKA6(1000.0,900.0));
		Rt.Set2(fXXZ(0.5,1.0),   fdLLH(10.0,30.0));
		Rmax = Rt*100;  Rmin = Rt*0.01;  Rb = 0.6;
		FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF,  fINF9,  fINF6);
	}
}

void SGClbrt::SetFt(int nnq)
{
	sins.etm();
	Ft.SetMat3(0,0,sins.Maa), Ft.SetMat3(0,3,sins.Mav), Ft.SetMat3(0,6,sins.Map), Ft.SetMat3(0,9,-sins.Cnb); 
	Ft.SetMat3(3,0,sins.Mva), Ft.SetMat3(3,3,sins.Mvv), Ft.SetMat3(3,6,sins.Mvp), Ft.SetMat3(3,12,sins.Cnb); 
						NULL, Ft.SetMat3(6,3,sins.Mpv), Ft.SetMat3(6,6,sins.Mpp);
	Ft.SetDiagVect3( 9, 9, sins._betaGyro);
	Ft.SetDiagVect3(12,12, sins._betaAcc);  // 0-14 phi,dvn,dpos,eb,db
}

void SGClbrt::SetHk(int nnq)
{
	if(nnq>=18) {     // GNSS lever
		Mat3 CW=sins.Cnb*askew(sins.webbar), MC=sins.Mpv*sins.Cnb;
		Hk.SetMat3(0,15, -CW);
		Hk.SetMat3(3,15, -MC);
	}
	if(nnq>=19) {    // GNSS dt
		Vect3 MV=sins.Mpv*sins.vn;
//		Hk.SetClmVect3(0,18, -sins.anbar);
		Hk.SetClmVect3(3,18, -MV);
	}
}

void SGClbrt::Feedback(int nnq, double fbts)
{
	Kalman::Feedback(nq, fbts);
	sins.qnb -= *(Vect3*)&FBXk.dd[0];  sins.vn -= *(Vect3*)&FBXk.dd[ 3];  sins.pos -= *(Vect3*)&FBXk.dd[6];
	sins.eb  += *(Vect3*)&FBXk.dd[9];	sins.db += *(Vect3*)&FBXk.dd[12];  // 0-14 phi,dvn,dpos,eb,db
	if(nnq>=18) {
		lvGNSS += *(Vect3*)&FBXk.dd[15];	// 15-17 lever
	}
	if(nnq>=19) {
		dtGNSSdelay += FBXk.dd[18];			// 18 dt
	}
}

void SGClbrt::SetMeasGNSS(const Vect3 &posgnss, const double &vbgnss, double yawgnss, double qfactor)
{
	if(!IsZero(posgnss) && avpi.Interp(posGNSSdelay+dtGNSSdelay,0x4))
	{
		*(Vect3*)&Zk.dd[3] = avpi.pos - posgnss;
		SetMeasFlag(00070);
	}
	if(!(vbgnss<EPS&&vbgnss>-EPS) && avpi.Interp(vnGNSSdelay+dtGNSSdelay,0x2))
	{
		Vect3 vngnss;
		Mat3 Cnb = a2mat(avpi.att);
		vngnss.x = Cnb.e01*vbgnss;
		vngnss.y = Cnb.e11*vbgnss;
		vngnss.z = Cnb.e21*vbgnss;

		*(Vect3*)&Zk.dd[0] = avpi.vn - vngnss;
		SetMeasFlag(00007);
	}
	if(!IsZero(yawgnss) && avpi.Interp(yawGNSSdelay+dtGNSSdelay,0x1))
	{
		Zk.dd[6] = -diffYaw(avpi.att.z, yawgnss+dyawGNSS);
		SetMeasFlag(00100);
	}
}

int SGClbrt::TDUpdate(const Vect3 *pwm, const Vect3 *pvm, int nSamples, double ts, int nStep)
{
	sins.Update(pwm, pvm, nSamples, ts);
	Feedback(nq, sins.nts);
	for(int j=0; j<nr; j++) {
		measlost.dd[j] += sins.nts;
		if(measstop.dd[j]>0.0) measstop.dd[j] -= sins.nts;
	}

	measRes = 0;

	if(nStep<=0||nStep>=maxStep) { nStep=maxStep; }
	tdts += sins.nts; kftk = sins.tk;  kfcount++;
	meanfn = meanfn+sins.fn; ifn++;
	for(int i=0; i<nStep; i++)
	{
		if(iter==-2)			// -2: set measurements
		{
			if(ifn==0)	break;
			Vect3 fn=sins.fn, an=sins.an;
			sins.fn = meanfn*(1.0/ifn); meanfn = O31; ifn = 0;
			sins.an = sins.fn+sins.eth.gcc; 
			SetFt(nq);
			SetMeas(); SetHk(nq); sins.fn = fn; sins.an = an;
		}
		else if(iter==-1)			// -1: discrete
		{
			Fk = ++(Ft*tdts); // Fk = I+Ft*ts
			Qk = Qt*tdts; 
			Xk = Fk*Xk;
			meantdts = tdts; tdts = 0.0;
		}
		else if(iter<nq)		// 0 -> (nq-1): Fk*Pk
		{
			int row=iter;
			RowMul(Pk1, Fk, Pk, row);
		}
		else if(iter<2*nq)		// nq -> (2*nq-1): Fk*Pk*Fk+Qk
		{
			int row=iter-nq;
			RowMulT(Pk, Pk1, Fk, row);
			Pk.dd[nq*row+row] += Qk.dd[row];
//			if(row==nq-1) {	Pk += Qk; }
		}
		else if(iter<2*(nq+nr))	// (2*nq) -> (2*(nq+nr)-1): sequential measurement updating
		{
			int row=(iter-2*Ft.row)/2;
			int flag = (measflag&measmask)&(0x01<<row);
			if(flag)
			{
//				if((iter-2*Ft.row)%2==0)
				if(iter%2==0)
				{
					Hi = Hk.GetRow(row);
					Pxz = Pk*(~Hi);
					Pz0 = (Hi*Pxz)(0,0);
					innovation = Zk(row)-(Hi*Xk)(0,0);
					adptOKi = 1;
					if(Rb.dd[row]>EPS)
						adptOKi=RAdaptive(row, innovation, Pz0);
					double Pzz = Pz0 + Rt(row)/rts(row);
					Kk = Pxz*(1.0/Pzz);
				}
				else
				{
					measflag ^= flag;
					if(adptOKi && measstop.dd[row]<EPS && (innovation>-Zmax.dd[row]&&innovation<Zmax.dd[row]))
					{
						measRes |= flag;
						Xk += Kk*innovation;
						Pk -= Kk*(~Pxz);
						measlost.dd[row] = 0.0;
					}
				}
			}
			else
			{
				nStep++;
			}
			if(iter%2==0)
				RtFading(row, meantdts);
		}
		else if(iter==2*(nq+nr))	// 2*(nq+nr): Xk,Pk constrain & symmetry
		{
			XPConstrain();
		}
		else if(iter>=2*(nq+nr)+1)	// 2*(nq+nr)+1: Miscellanous
		{
			iter = -3;
		}
		iter++;
	}

	measflaglog |= measRes;
	return measRes;
}

int SGClbrt::Update(const Vect3 *pwm, const Vect3 *pvm, int nn, double ts, int nSteps)
{
	int res=TDUpdate(pwm, pvm, nn, ts, nSteps);
	sins.lever(lvGNSS);
	avpi.Push(sins, 1);
	return res;
}


BOOL SGClbrt::IsZero(const Vect3 &v, double eps)
{
	return (v.x<eps&&v.x>-eps && v.y<eps&&v.y>-eps && v.z<eps&&v.z>-eps);
}


