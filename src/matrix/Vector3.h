/**
 * @file Vector3.h
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

class Vect3
{
public:
    double x, y, z;

public:
    Vect3(void);
    Vect3(double xyz);
    Vect3(double x, double y, double z);

    Vect3& operator=(double f);             // 每一个元素都等于一个数
    Vect3& operator=(const double *pf);     // 通过数组赋值向量
};

