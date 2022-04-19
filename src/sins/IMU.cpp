/**
 * @file IMU.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "IMU.h"

extern const Vect3	O31;

IMU::IMU(void)
{
	preFirst = onePlusPre = true; 
	phim = dvbm = wm_1 = vm_1 = O31;
}

void IMU::Update(const Vect3 *pwm, const Vect3 *pvm)
{
	Vect3 cm(0.0), sm(0.0); wmm=O31, vmm=O31;

	if(onePlusPre)  // one-plus-previous sample
	{
		if(preFirst) { wm_1=pwm[0]; vm_1=pvm[0]; preFirst=false; }
		cm = 1.0/12*wm_1;
		sm = 1.0/12*vm_1;
	}
	wm_1=pwm[0];  vm_1=pvm[0];
	wmm += pwm[0];
	vmm += pvm[0];
	phim = wmm + cm*pwm[0];
	dvbm = vmm + 1.0/2*wmm*vmm + (cm*pvm[0]+sm*pwm[0]);
}

