//
// Created by insppp on 2022/8/5.
//

#ifndef MY_FC_FW_MY_MATH_H
#define MY_FC_FW_MY_MATH_H
#include "math.h"
#include "struct_typedef.h"


#define squa(n) (float)n*(float)n


typedef volatile struct
{
	float q0;
	float q1;
	float q2;
	float q3;
} Quaternion;


#endif //MY_FC_FW_MY_MATH_H
