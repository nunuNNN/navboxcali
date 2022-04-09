/**
 * @file SINS.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "SINS.h"

extern const Vect3	O31;
extern const Mat3	I33, O33, One33;

SINS::SINS(const Vect3 &att0, const Vect3 &vn0, const Vect3 &pos0, double tk0)
{
	Init(a2qua(att0), vn0, pos0, tk0);
}
SINS::SINS(const Quat &qnb0, const Vect3 &vn0, const Vect3 &pos0, double tk0)
{
	Init(qnb0, vn0, pos0, tk0);
}

void SINS::Init(const Quat &qnb0, const Vect3 &vn0, const Vect3 &pos0, double tk0)
{
	tk = tk0;  ts = nts = 1.0;

	velMax = 400.0; 
	hgtMin = -RE*0.01, 
	hgtMax = -hgtMin; 
	afabar = 0.1;

	qnb = qnb0;	
	vn = vn0, 
	pos = pos0;
	Cbn = I33;

	Kg = Ka = I33; 
	eb = db = Ka2 = O31;

	Maa = Mav = Map = Mva = Mvv = Mvp = Mpv = Mpp = O33;

	lvr = O31; an = anbar = webbar = O31;

	Vect3 wib(0.0), fb=(~qnb)*Vect3(0,0,glv.g0);
	isOpenloop = 0;

	Update(&wib, &fb, 1, 1.0); imu.preFirst = 1;

	tk = tk0;  ts = nts = 1.0; 
	qnb = qnb0;	vn = vn0, pos = pos0;
	
	etm(); lever();
}

void SINS::Update(const Vect3 *pwm, const Vect3 *pvm, int nSamples, double ts)
{
	this->ts = ts;  nts = nSamples*ts;	tk += nts;
	double nts2 = nts/2;
	imu.Update(pwm, pvm);
	imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
	imu.dvbm.x -= Ka2.x*imu.dvbm.x*imu.dvbm.x/nts;
	imu.dvbm.y -= Ka2.y*imu.dvbm.y*imu.dvbm.y/nts;
	imu.dvbm.z -= Ka2.z*imu.dvbm.z*imu.dvbm.z/nts;
//		Vect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
	if(!isOpenloop) eth.Update(pos,O31);
	wib = imu.phim/nts; fb = imu.dvbm/nts;
	web = wib;  webbar = (1-afabar)*webbar + afabar*web;
	wnb = wib;
	fn = qnb*fb;
	an = fn+eth.gcc;  anbar = (1-afabar)*anbar + afabar*an;
	Vect3 vn1 = vn + an*nts;
	pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
	qnb = qnb*rv2q(imu.phim);
	Cnb = q2mat(qnb); att = m2att(Cnb); Cbn = ~Cnb; vb = Cbn*vn;


	if(vn.x>velMax) vn.x=velMax; else if(vn.x<-velMax) vn.x=-velMax;
	if(vn.y>velMax) vn.y=velMax; else if(vn.y<-velMax) vn.y=-velMax;
	if(vn.z>velMax) vn.z=velMax; else if(vn.z<-velMax) vn.z=-velMax;

	if(pos.x>89.9*DEG) pos.x=89.9*DEG; else if(pos.x<-89.9*DEG) pos.x=-89.9*DEG;
	if(pos.y>PI) pos.y-=_2PI; else if(pos.y<-PI) pos.y+=_2PI;
	if(pos.z>hgtMax) pos.z=hgtMax; else if(pos.z<hgtMin) pos.z=hgtMin;

}

void SINS::lever(const Vect3 &dL, Vect3 *ppos, Vect3 *pvn)
{
	if(&dL!=&O31) lvr = dL;
//	Mpv = Mat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
	Mpv.e01=eth.f_RMh, Mpv.e10=eth.f_clRNh, Mpv.e22=1.0;
	CW = Cnb*askew(web), MpvCnb = Mpv*Cnb;
	if(ppos==NULL) {
		posL = pos + MpvCnb*lvr;
		if(pvn==NULL)  vnL = vn + CW*lvr;
	} 
	else {
		*ppos = pos + MpvCnb*lvr;
		if(pvn!=NULL)  *pvn = vn + CW*lvr;
	}
}

void SINS::etm(void)
{
	Mva = askew(fn);
	Mpv = Mat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
}

