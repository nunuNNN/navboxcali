/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "IOfile.h"
#include "SGClbrt.h"


#include <fcntl.h> 
#include <unistd.h>

#define disp(i, FRQ, n)  { if((i)%((n)*(FRQ))==0) printf("%d\n", (i)/(FRQ));} 

#ifdef PSINS_IO_FILE
class CFileIMUGNSS:public CFileRdWt  // read the 13-column IMU/GPS bin file created by 'binfile.m'
{
public:
	Vect3 *pwm, *pvm, *pvGNSS, *ppGNSS, vn0, pos0;
	double t0, ts, t, dt;
	CFileIMUGNSS(const char *fname0):CFileRdWt(fname0, -14) {
		pwm=(Vect3*)&buff[0], pvm=(Vect3*)&buff[3], pvGNSS=(Vect3*)&buff[7], ppGNSS=(Vect3*)&buff[10];
		CFileRdWt::load(1);	vn0=*pvGNSS, pos0=*ppGNSS, t0=buff[6];
		CFileRdWt::load(1);	t=buff[6]; ts=t-t0;
		if(IsZero(vn0)) {
			while(IsZero(*pvGNSS)) if(!load(1)) break;  // wait for first GNSS
			vn0=*pvGNSS, pos0=*ppGNSS;
		}
	};
	int load(int lines=1) {
		if(!CFileRdWt::load(lines)) return 0;
		t += ts;
		dt = buff[13];
		return 1;
	};

    BOOL IsZero(const Vect3 &v, double eps=EPS){
        return (v.x<eps&&v.x>-eps && v.y<eps&&v.y>-eps && v.z<eps&&v.z>-eps);
    };
};
#endif

int main(void)
{
	CFileRdWt::Dir("/mnt/c/Users/HP/Desktop/compare/");
	CFileIMUGNSS fimu("imugps.bin"); CFileRdWt fins("ins.bin"), fkf("kf.bin");

	SGClbrt kf(19, 6, fimu.ts);
	kf.Init(SINS(Vect3(0,0,67)*glv.deg,fimu.vn0,fimu.pos0,fimu.t));
	kf.Pk.SetDiag2(fPHI(60,600),  fXXX(1.0),  fdPOS(100.0),  fDPH3(1000),  fMG3(10.0),
			fXXX(1.0),  0.01,  fdKG9(100.0,90.0), fdKA6(10.0,10.0));
	kf.Pmax = INF;
	kf.Pmin.Set2(fPHI(0.1,1.0),  fXXX(0.01),  fdPOS(0.1),  fDPH3(1),  fUG3(100.0),
			fXXX(0.001), 0.0001, fdKG9(1.0,1.0), fdKA6(1.0,1.0));
	kf.Qt.Set2(fDPSH3(1),  fUGPSHZ3(100.0),  fOOO,  fOO6,
			fOOO, 0.0,  fOO9,  fOO6);
	kf.Xmax = INF;
	kf.Rt.Set2(fXXZ(0.08,0.2),   fdLLH(0.15, 0.3));
	kf.Rmax = kf.Rt*100;  kf.Rmin = kf.Rt*0.01;  kf.Rb = 0.2;
	kf.FBTau.Set(fXX9(0.1),  fXX6(1.0),  fINF3, INF,  fINF9,  fINF6);

	for(int i=0; i<5000*100; i++)
	{
		if(!fimu.load(1)) break;
		kf.Update(fimu.pwm, fimu.pvm, 1, fimu.ts);
		if(fimu.ppGNSS->x>0.1)
			kf.SetMeasGNSS(*fimu.ppGNSS, *fimu.pvGNSS);

		if(i%20==0 || fimu.ppGNSS->x>0.1)
			fins<<kf.sins<<*fimu.pvGNSS<<*fimu.ppGNSS;
		if(i%20==0) fkf<<kf;
		disp(i, 100, 100);

		// usleep(1000 * 100);
	}

}