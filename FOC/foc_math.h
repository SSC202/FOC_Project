/**
 * @file    数学计算和通用计算相关
 * @note    本文件包含数学常量定义,通用计算的相关定义和基础数学运算的相关定义。
 *          1. 通用计算相关定义
 *              接口函数:    get_max()       三值取最大值函数
 *                          get_min()       三值取最小值函数
 *                          get_middle()    三值取中间值函数
 *                          normalize()     机械角度转换为电角度
 *                          fast_sin()      快速正弦函数
 *                          fast_cos()      快速余弦函数
 *                          fast_sqrt()     快速开方函数
 *          2. PID 计算(增量式PID)
 *              接口类型: PID_t  PID结构体句柄,使用时在主函数内定义全局结构体
 *              接口函数:   PID_init()      PID结构体初始化
 *                          PID_Calc()     单次PID计算
 *          3. IIR 滤波器
 *              接口类型: LPF_t  低通滤波器结构体句柄,使用时在主函数内定义全局结构体
 *              接口函数:   LPF_Init()      低通滤波器结构体初始化
 *                          LPF_Calc()     单次低通滤波器计算
 *      
 */
#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "math.h"
#include "stm32h7xx.h"

#define M_PI         3.141592653589793f // PI
#define M_TABLE_SIZE 1024

float normalize(int pole_pairs, float mechine_angle, float offset);
float get_middle(float a, float b, float c);
float get_max(float a, float b, float c);
float get_min(float a, float b, float c);

float fast_sin(float x);
float fast_cos(float x);
float fast_sqrt(float x);

#endif