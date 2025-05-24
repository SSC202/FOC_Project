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

/**
 * IIR 滤波器相关定义
 */
/**
 * @brief   一阶低通滤波器
 */
typedef struct
{
    float tsample;     // 采样时间
    float wc;          // 截止频率
    float alpha;       // 滤波器系数
    float input;       // 当前输入值
    float output_last; // 前次输出值
    float output;      // 当前输出值
} LPF_t;

/**
 * @note 巴特沃斯型高通滤波器,可选择阶数和离散化方法
 * @note 根据 filter_type 和 wc 计算对应的滤波器参数a0,a1,a2
 * @note ******************************************************
 * @note type = 1 : 一阶，后向差分法
 * @note ******************************************************
 * @note type = 2 : 二阶，后向差分法
 * @note ******************************************************
 * @note type = 3 : 一阶，Tustin法
 * @note ******************************************************
 * @note type = 4 : 二阶，Tustin法
 */
typedef struct
{
    float tsample; // 采样时间
    float wc;      // 截止频率

    uint8_t type; // 滤波器类型选择系数

    float a1; // 滤波器系数（输出）
    float a2; // 滤波器系数
    float b0; // 滤波器系数（输入）
    float b1; // 滤波器系数
    float b2; // 滤波器系数

    float input_last[2];  // 前两次输入值，0：上上次，1：上次
    float input;          // 当前输入值
    float output_last[2]; // 前两次输出值，0：上上次，1：上次
    float output;         // 当前输出值
} HPF_t;

/**
 * 指令生成定义
 */
// 斜坡指令生成
typedef struct
{
    float tsample; // 采样时间
    float K_rise;  // 斜率
    float gap;     // 判断达到给定值的条件
    float ref_in;  // 指令输入
    float ref_out; // 当前模块输出
} Slope_t;

float normalize(int pole_pairs, float mechine_angle, float offset);
float get_middle(float a, float b, float c);
float get_max(float a, float b, float c);
float get_min(float a, float b, float c);

void PID_init(PID_t *pid, float kp, float ki, float kd, float outputMax);
void PID_Calc(PID_t *pid, uint8_t enable, float t_sample);

void LPF_Init(LPF_t *lpf, float f_c, float t_sample);
void LPF_Calc(LPF_t *lpf, uint8_t enable);

void HPF_Init(HPF_t *hpf, float t_sample, float f_c, uint8_t type);
void HPF_Calc(HPF_t *hpf, uint8_t enable);
void HPF_Design(HPF_t *hpf, float f_c, uint8_t type, float t_sample);

float fast_sin(float x);
float fast_cos(float x);
float fast_sqrt(float x);

void Slope_Module(Slope_t *slope, float t_sample, uint8_t enable);
void Slope_Init(Slope_t *slope, float t_sample, float gap, float krise);

#endif