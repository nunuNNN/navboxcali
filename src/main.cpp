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

#include <fstream>
#include <sstream>
#include <iostream>
#include <string>

#include <fcntl.h> 
#include <unistd.h>

#define disp(i, FRQ, n)  { if((i)%((n)*(FRQ))==0) printf("%d\n", (i)/(FRQ));} 

BOOL IsZero(const Vect3 &v, double eps=EPS)
{
	return (v.x<eps&&v.x>-eps && v.y<eps&&v.y>-eps && v.z<eps&&v.z>-eps);
}

int main(void)
{
	CFileRdWt::Dir("/mnt/e/project/navboxcali/data/");
	CFileRdWt fins("ins.bin"), fkf("kf.bin");

	std::string data_path = "./../data/imugps.txt";
	std::ifstream fsImuGps;
    fsImuGps.open(data_path.c_str(), std::ios::binary);
    if (!fsImuGps.is_open())
    {
        std::cerr << "Failed to open file! " << data_path << std::endl;
        return -1;
    }

	std::string sImuGps_line;
	Vect3 wm, vm, pGNSS;
	double vGNSS, hGNSS;
	double ts = 0.01, t, dt;
	SGClbrt kf(19, 6, ts);
    while (std::getline(fsImuGps, sImuGps_line) && !sImuGps_line.empty()) // read imu data
    {
        std::istringstream ssImuData(sImuGps_line);
        ssImuData >> wm.x >> wm.y >> wm.z >> vm.x >> vm.y >> vm.z 
				>> t >> vGNSS >> pGNSS.x >> pGNSS.y >> pGNSS.z >> hGNSS >> dt;

		if(!IsZero(pGNSS))
		{
			kf.Init(SINS(Vect3(0,0,67)*glv.deg, vGNSS, pGNSS, t));
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
			break;
		}
    }

	int i=0;
    while (std::getline(fsImuGps, sImuGps_line) && !sImuGps_line.empty()) // read imu data
    {
		i++;
        std::istringstream ssImuData(sImuGps_line);
        ssImuData >> wm.x >> wm.y >> wm.z >> vm.x >> vm.y >> vm.z 
				>> t >> vGNSS >> pGNSS.x >> pGNSS.y >> pGNSS.z >> hGNSS >> dt;
        std::cout << wm.x << ", "  << wm.y << ", "  << wm.z << ", " 
					<< vm.x << ", " << vm.y << ", " << vm.z << ", " 
					<< t << ", "  << vGNSS << ", " 
					<< pGNSS.x << ", "  << pGNSS.y << ", " << pGNSS.z << ", " 
					<< hGNSS << ", "  << dt << std::endl;

		kf.Update(&wm, &vm, 1, ts);
		if(pGNSS.x>0.1)
			kf.SetMeasGNSS(pGNSS, vGNSS);

		if(i%20==0 || pGNSS.x>0.1)
			fins<<kf.sins<<vGNSS<<pGNSS;
		if(i%20==0) fkf<<kf;
		disp(i, 100, 100);

        usleep(1000 * 500);
    }

    fsImuGps.close();

}



