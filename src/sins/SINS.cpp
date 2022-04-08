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

SINS::SINS(const Vect3 &att0, const Vect3 &vn0, const Vect3 &pos0, double tk0)
{
	Init(a2qua(att0), vn0, pos0, tk0);
}

void SINS::Init(const Quat &qnb0, const Vect3 &vn0, const Vect3 &pos0, double tk0)
{
	tk = tk0;  ts = nts = 1.0;
	// sins门限值
	velMax = 400.0; 
	hgtMin = -RE*0.01, 
	hgtMax = -hgtMin; 
	afabar = 0.1;

	// sins初始化
	qnb = qnb0;	
	vn = vn0, 
	pos = pos0;
	Cbn = I33;

	Kg = Ka = I33; 
	eb = db = Ka2 = O31;

	// 递推过程中协方差
	Maa = Mav = Map = Mva = Mvv = Mvp = Mpv = Mpp = O33;

	lvr = O31; an = anbar = webbar = O31;

	Vect3 wib(0.0), fb=(~qnb)*Vect3(0,0,glv.g0);
	isOpenloop = 0;

	Update(&wib, &fb, 1, 1.0); imu.preFirst = 1;

	tk = tk0;  ts = nts = 1.0; 
	qnb = qnb0;	vn = vn0, pos = pos0;
	
	etm(); lever();
}

void SINS::Update(const CVect3 *pwm, const CVect3 *pvm, int nSamples, double ts)
{
	this->ts = ts;  nts = nSamples*ts;	tk += nts;
	double nts2 = nts/2;
	imu.Update(pwm, pvm, nSamples);
	imu.phim = Kg*imu.phim - eb*nts; imu.dvbm = Ka*imu.dvbm - db*nts;  // IMU calibration
	imu.dvbm.i -= Ka2.i*imu.dvbm.i*imu.dvbm.i/nts;
	imu.dvbm.j -= Ka2.j*imu.dvbm.j*imu.dvbm.j/nts;
	imu.dvbm.k -= Ka2.k*imu.dvbm.k*imu.dvbm.k/nts;
//		CVect3 vn01 = vn+an*nts2, pos01 = pos+eth.vn2dpos(vn01,nts2);
	if(!isOpenloop) eth.Update(pos,O31,1);
	wib = imu.phim/nts; fb = imu.dvbm/nts;
	web = wib;  webbar = (1-afabar)*webbar + afabar*web;
	wnb = wib;
	fn = qnb*fb;
	an = fn+eth.gcc;  anbar = (1-afabar)*anbar + afabar*an;
	CVect3 vn1 = vn + an*nts;
	pos = pos + eth.vn2dpos(vn+vn1, nts2);	vn = vn1;
	qnb = qnb*rv2q(imu.phim);
	Cnb = q2mat(qnb); att = m2att(Cnb); Cbn = ~Cnb; vb = Cbn*vn;


	if(vn.i>velMax) vn.i=velMax; else if(vn.i<-velMax) vn.i=-velMax;
	if(vn.j>velMax) vn.j=velMax; else if(vn.j<-velMax) vn.j=-velMax;
	if(vn.k>velMax) vn.k=velMax; else if(vn.k<-velMax) vn.k=-velMax;

	if(pos.i>89.9*DEG) pos.i=89.9*DEG; else if(pos.i<-89.9*DEG) pos.i=-89.9*DEG;
	if(pos.j>PI) pos.j-=_2PI; else if(pos.j<-PI) pos.j+=_2PI;
	if(pos.k>hgtMax) pos.k=hgtMax; else if(pos.k<hgtMin) pos.k=hgtMin;

}

void SINS::lever(const CVect3 &dL, CVect3 *ppos, CVect3 *pvn)
{
	if(&dL!=&O31) lvr = dL;
//	Mpv = CMat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
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
	Mpv = CMat3(0,eth.f_RMh,0, eth.f_clRNh,0,0, 0,0,1);
}


