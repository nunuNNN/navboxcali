/**
 * @file Kalman.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Kalman.h"

Kalman::Kalman(void)
{
}

Kalman::Kalman(int nq0, int nr0)
{
//	psinsassert(nq0<=MMD&&nr0<=MMD);
	if(!(nq0<=MMD&&nr0<=MMD)) { /*printf("\tMMD too small!\n");*/ exit(0); }
	Init(nq0, nr0);
}

void Kalman::Init(int nq0, int nr0)
{
	kftk = 0.0;
	nq = nq0; nr = nr0;
	Ft = Pk = Mat(nq,nq,0.0);
	Hk = Mat(nr,nq,0.0);  Fading = Mat(nr,nq,1.0); zfdafa = 0.1;
	Qt = Pmin = Xk = Vect(nq,0.0);  Xmax = Pmax = Vect(nq,INF);  Pset = Vect(nq,-INF);
	Zk = Vect(nr,0.0);  Rt = Vect(nr,INF); rts = Vect(nr,1.0);  Zfd = Vect(nr,0.0); Zfd0 = Zmax = Vect(nr,INF);
	RtTau = Rmax = Vect(nr,INF); measstop = measlost = Rmin = Rb = Rstop = Vect(nr,0.0); Rbeta = Vect(nr,1.0);
	SetRmaxcount(5);
	FBTau = FBMax = FBOne = FBOne1 = Vect(nq,INF); FBXk = FBTotal = Vect(nq,0.0);
	kfcount = measflag = measflaglog = 0;  SetMeasMask(3,nr0);
}

void Kalman::SetRmaxcount(int cnt)
{
	for(int i=0; i<nr; i++) { Rmaxcount[i]=0, Rmaxcount0[i]=cnt; }
}

void Kalman::SetMeasFlag(unsigned int flag)
{
	measflag = (flag==0) ? 0 : (measflag|flag);
}

void Kalman::SetMeasMask(int type, unsigned int mask)
{
	int m;
	if(type==1) measmask = mask;		// set mask 1
	else if(type==0) measmask &= ~mask;	// set mask 0
	else if(type==2) measmask |= mask;	// add mask 1
	else if(type==3) {					// set mask-LSB 1
		for(m=0; mask>0; mask--)  m |= 1<<(mask-1);
		SetMeasMask(1, m);
	}
}

void Kalman::SetMeasStop(double stop, unsigned int meas)
{
	measstop.SetBit(meas, stop);
}

void Kalman::SetRadptStop(double stop, unsigned int meas)
{
	Rstop.SetBit(meas, stop);
}

int Kalman::RAdaptive(int i, double r, double Pr)
{
	double rr=r*r-Pr;
	if(rr<Rmin.dd[i])	rr = Rmin.dd[i];
	if(rr>Rmax.dd[i])	{ Rt.dd[i]=Rmax.dd[i]; Rmaxcount[i]++; }  
	else				{ Rt.dd[i]=(1.0-Rbeta.dd[i])*Rt.dd[i]+Rbeta.dd[i]*rr; Rmaxcount[i]=0; }
	Rbeta.dd[i] = Rbeta.dd[i]/(Rbeta.dd[i]+Rb.dd[i]);   // beta = beta / (beta+b)
	int adptOK = (Rmaxcount[i]==0||Rmaxcount[i]>Rmaxcount0[i]) ? 1: 0;
	return adptOK;
}

void Kalman::XPConstrain(void)
{
	int i=0, nq1=nq+1;
	for(double *px=Xk.dd,*pxmax=Xmax.dd,*p=Pk.dd,*pmin=Pmin.dd,*pminEnd=&Pmin.dd[nq],*pmax=Pmax.dd,*pset=Pset.dd;
		pmin<pminEnd; px++,pxmax++,p+=nq1,pmin++,pmax++,pset++)
	{
		if(*px>*pxmax)		// Xk constrain
		{
			*px = *pxmax;
		}
		else if(*px<-*pxmax)
		{
			*px = -*pxmax;
		}
		if(*p<*pmin)		// Pk constrain
		{
			*p = *pmin;
		}
		else if(*p>*pmax)
		{
			double sqf=sqrt(*pmax/(*p))*0.9;
			for(double *prow=&Pk.dd[i*Pk.clm],*prowEnd=prow+nq,*pclm=&Pk.dd[i]; prow<prowEnd; prow++,pclm+=nq)
			{
				*prow *= sqf;
				*pclm = *prow;
			}
			Pk.dd[i*Pk.clm+i] *= sqf;  //20200303
			break;
		}
		if(*pset>0.0)	// Pk set
		{
			if(*p<*pset)
			{
				*p = *pset;
			}
			else if(*p>*pset)
			{
				double sqf=sqrt(*pset/(*p));
				for(double *prow=&Pk.dd[i*Pk.clm],*prowEnd=prow+nq,*pclm=&Pk.dd[i]; prow<prowEnd; prow++,pclm+=nq)
				{
					*prow *= sqf;
					*pclm *= sqf;
				}
			}
			*pset = -1.0;
		}
		i++;
	}
}

void Kalman::PmaxPminCheck(void)
{
	for(double *p=Pk.dd,*pmax=Pmax.dd,*pmin=Pmin.dd,*pminEnd=&Pmin.dd[nq]; pmin<pminEnd; p+=nq+1,pmax++,pmin++)
	{
		if(*p>*pmax) *pmax = *p*10.0;
		if(*p<EPS)	 *pmin = 0.0;  else if(*p<*pmin)  *pmin = *p/2.0;
	}
}

void Kalman::Feedback(int nnq, double fbts)
{
	double *pTau=FBTau.dd, *pTotal=FBTotal.dd, *pMax=FBMax.dd, *pOne=FBOne.dd, *pOne1=FBOne1.dd, *pXk=FBXk.dd, *p=Xk.dd;
	for(int i=0; i<nq; i++, pTau++,pTotal++,pMax++,pOne++,pOne1++,pXk++,p++)
	{
		if(*pTau<INFp5)
		{
			if(*p>*pOne1 || *p<-*pOne1) {  // max feedback/one step
				*pXk=*p;
			}
			else {
				double afa = fbts<*pTau ? fbts/(*pTau) : 1.0;
				*pXk = *p*afa;
				if(*pXk>*pOne) *pXk=*pOne; else if(*pXk<-*pOne) *pXk=-*pOne;  // min feedback/one step
			}
			if(*pMax<INFp5)
			{
				if(*pTotal+*pXk>*pMax)			*pXk = *pMax-*pTotal;
				else if(*pTotal+*pXk<-*pMax)	*pXk = -*pMax-*pTotal;
			}
			*p -= *pXk;
			*pTotal += *pXk;
		}
		else
		{
			*pXk = 0.0;
		}
	}
}

void Kalman::RtFading(int i, double fdts)
{
	double Taui=RtTau.dd[i], Rti=Rt.dd[i], Rmaxi=Rmax.dd[i];
	if(measlost.dd[i]>3.0 && Taui<INFp5 && Rb.dd[i]>0.0 && Rti<Rmaxi)
	{
		double afa = fdts<Taui ? fdts/Taui : 1.0;
		Rti += 2*sqrt(Rmaxi*Rti)*afa;
		Rt.dd[i] = Rti;
	}
}



