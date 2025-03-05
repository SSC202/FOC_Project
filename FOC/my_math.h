/**
 * @file    数学计算和通用计算相关
 * @note    本文件包含数学常量定义,通用计算的相关定义和基础数学运算的相关定义。
 *          1. 通用计算相关定义
 *              get_max()       三值取最大值函数
 *              get_min()       三值取最小值函数
 *              get_middle()    三值取中间值函数
 *
 *              normalize()     角度归一化函数,将任意范围的角度归一化到-PI到+PI内
 *          2. 基础数学运算定义
 *              1. 接口
 *                  PID_t       增量式 PID 控制器结构体
 *              2. 函数
 *                  PID_Init()  增量式 PID 控制器初始化
 *                  PID_Calc()  增量式 PID 控制器单次计算
 */
#ifndef __MY_MATH_H
#define __MY_MATH_H

#include "math.h"
#include "stm32h7xx.h"

#define M_PI     3.141592653589793f // PI

/**
 * PID 相关定义
 */
typedef struct {
    float KP;        // PID参数P
    float KI;        // PID参数I
    float KD;        // PID参数D
    float fdb;       // PID反馈值
    float ref;       // PID目标值
    float cur_error; // 当前误差
    float error[2];  // 前两次误差
    float output;    // 输出值
    float outputMax; // 最大输出值的绝对值
    float outputMin; // 最小输出值的绝对值用于防抖
} PID_t;

float normalize(int pole_pairs, float mechine_angle, float offset);
float get_middle(float a, float b, float c);
float get_max(float a, float b, float c);
float get_min(float a, float b, float c);

void PID_init(PID_t *pid, float kp, float ki, float kd, float outputMax);
void PID_Calc(PID_t *pid, uint8_t enable, float t_sample);

#endif