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

	// 
	Kg = Ka = I33; 
	eb = db = Ka2 = O31;

	// 递推过程中协方差
	Maa = Mav = Map = Mva = Mvv = Mvp = Mpv = Mpp = O33;

	lvr = O31;

	Vect3 wib(0.0), fb=(~qnb)*Vect3(0,0,glv.g0);
	an = anbar = webbar = O31;
	isOpenloop = 0;

	Update(&wib, &fb, 1, 1.0); imu.preFirst = 1;

	tk = tk0;  ts = nts = 1.0; 
	qnb = qnb0;	vn = vn0, pos = pos0;
	
	mmwb = mmfb = CMaxMin(100);
	mvn = mvnmax = mvnmin = vn; mvni = O31; mvnt = mvnT = 0.0; mvnk = 0;
	etm(); lever(); Extrap();
}





