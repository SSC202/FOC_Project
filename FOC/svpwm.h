/**
 * @file    SVPWM 相关代码
 * @note    本文件包含 SVPWM 的相关定义
 *          1. 接口
 *              duty_abc_t  ABC 三相占空比
 *          2. 函数定义
 *              e_svpwm()   简单 SVPWM (三次谐波注入法)
 */
#ifndef __SVPWM_H
#define __SVPWM_H

#include "coordinate_transform.h"
#include "my_math.h"

typedef struct
{
    float dutya;
    float dutyb;
    float dutyc;
} duty_abc_t;

void e_svpwm(abc_t *u_ref, float u_dc, duty_abc_t *duty_abc);

#endif // !_SVPWM_H__
