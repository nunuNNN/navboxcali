/**
 * @file AVPInterp.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "AVPInterp.h"

AVPInterp::AVPInterp(void)
{
}

void AVPInterp::Init(const SINS &sins, double ts, BOOL islever)
{
	if(islever)
		Init(sins.att, sins.vnL, sins.posL, ts);
	else
		Init(sins.att, sins.vn, sins.pos, ts);
}

void AVPInterp::Init(const Vect3 &att0, const Vect3 &vn0, const Vect3 &pos0, double ts)
{
	this->ts = ts;
	ipush = 0;
	for(int i=0; i<AVPINUM; i++) { atti[i]=att0, vni[i]=vn0; posi[i]=pos0; }
	att = att0, vn = vn0, pos = pos0;
}

void AVPInterp::Push(const SINS &sins, BOOL islever)
{
	if(islever)
		Push(sins.att, sins.vnL, sins.posL);
	else
		Push(sins.att, sins.vn, sins.pos);
}

void AVPInterp::Push(const Vect3 &attk, const Vect3 &vnk, const Vect3 &posk)
{
	if(++ipush>=AVPINUM) ipush = 0;
	atti[ipush] = attk; vni[ipush] = vnk; posi[ipush] = posk;
}

int AVPInterp::Interp(double tpast, int avp)
{
	int res=1, k, k1, k2;
	if(tpast<-AVPINUM*ts) tpast=-AVPINUM*ts; else if(tpast>0) tpast=0;
//	if(tpast<-AVPINUM*ts||tpast>0) return (res=0);
	k = (int)(-tpast/ts);
	if((k2=ipush-k)<0) k2 += AVPINUM;
	if((k1=k2-1)<0)  k1 += AVPINUM;
	double tleft = -tpast - k*ts;
	if(tleft>0.99*ts)	{
		if(avp&0x1) att=atti[k1]; if(avp&0x2) vn=vni[k1]; if(avp&0x4) pos=posi[k1];
	}
	else if(tleft<0.01*ts)	{
		if(avp&0x1) att=atti[k2]; if(avp&0x2) vn=vni[k2]; if(avp&0x4) pos=posi[k2];
	}
	else	{
		double b=tleft/ts, a=1-b;
		if(avp&0x1)	{ att = b*atti[k1]+a*atti[k2]; if(normInf(att-atti[k1])>10.0*DEG) res=0; }
		if(avp&0x2)	{ vn  = b*vni[k1] +a*vni[k2]; }
		if(avp&0x4)	{ pos = b*posi[k1]+a*posi[k2]; if(fabs(pos.y-posi[k1].y)>1.0*DEG) res=0; }
	}
	return res;
}


