/**
 * @file IMU.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#pragma once

#include <cstdio>
#include "CMath.hpp"

class IMU
{
public:
	bool preFirst, onePlusPre;
	Vect3 phim, dvbm, wmm, vmm, wm_1, vm_1;
public:
    IMU(void);
    void Update(const Vect3 *pwm, const Vect3 *pvm);
};

