/**
 * @file Glv.hpp
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

#pragma pack(4)
class GLV
{
public:
	double Re, f, g0, wie;										// the Earth's parameters
	double e, e2, ep, ep2, Rp;
	double mg, ug, deg, min, sec, hur, ppm, ppmpsh;					// commonly used units
	double dps, dph, dpsh, dphpsh, dph2, dphpg, ugpsh, ugpsHz, ugpg2, mpsh, mpspsh, secpsh;

	GLV(double Re=6378137.0, double f=(1.0/298.257), double wie0=7.2921151467e-5, double g0=9.7803267714)
    {
        this->Re = Re; this->f = f; this->wie = wie0; this->g0 = g0;
        Rp = (1-f)*Re;
        e = sqrt(2*f-f*f); e2 = e*e;
        ep = sqrt(Re*Re-Rp*Rp)/Rp; ep2 = ep*ep;
        mg = g0/1000.0;
        ug = mg/1000.0;
        deg = PI/180.0;
        min = deg/60.0;
        sec = min/60.0;
        ppm = 1.0e-6;
        hur = 3600.0;
        dps = deg/1.0;
        dph = deg/hur;
        dpsh = deg/sqrt(hur);
        dphpsh = dph/sqrt(hur);
        dph2 = dph/hur;
        dphpg = dph/g0;
        ugpsHz = ug/sqrt(1.0);
        ugpsh = ug/sqrt(hur);
        ugpg2 = ug/g0/g0;
        mpsh = 1/sqrt(hur); 
        mpspsh = 1/1/sqrt(hur);
        ppmpsh = ppm/sqrt(hur);
        secpsh = sec/sqrt(hur);
    }
};
#pragma pack()


