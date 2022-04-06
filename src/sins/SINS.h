/**
 * @file SINS.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include <cstdio>
#include "CMath.hpp"
#include "Earth.h"
#include "IMU.h"

class SINS
{
public:
    double ts, nts, tk;
    
public:
    SINS(const Vect3 &att0, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);
    void Init(const Quat &qnb0=qI, const Vect3 &vn0=O31, const Vect3 &pos0=O31, double tk0=0.0);    // initialization using quat attitude, velocity & position

};


