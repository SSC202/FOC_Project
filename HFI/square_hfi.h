#ifndef __SQUARE_HFI_H
#define __SQUARE_HFI_H

#include "foc_math.h"
#include "coordinate_transform.h"
#include "user_transfunc.h"

typedef struct
{
    // Init Parameters 初始化相关参数
    float ish;               // 注入高频电流幅值
    float theta_inj_i;       // 电流注入角度
    float offset;            // 变轴系信号偏置
    float sample_time;       // 采样时间
    PID_t pll;               // 锁相环
    LPF_t speed_lpf;         // 速度低通滤波器
    PID_t ish_pi_controller; // 信噪比控制器

    // Control Parameters 运行控制相关参数
    uint8_t step;   // 运行步
    uint8_t enable; // 算法使能

    // Output Parameters 输出参数
    dq_t udq_h;      // 高频注入电压
    float theta_obs; // 估计角度
    float speed_obs; // 估计速度

    // Input Parameters 输入参数
    dq_t idqh_now;    // 高频注入响应电流
    float theta_true; // 真实角度
    float speed_true; // 真实速度

    // Temporary variables 临时变量
    float u_h;       // 注入电压
    float theta_inj; // 电压注入角度
    dq_t idq_h[4];   // 高频注入电流
    float isig;      // 解调后信号
    float icomp;     // 偏置信号
    float power;     // 高频功率
} HFI_t;

void HFI_Inject(HFI_t *hfi); // 电压注入
void HFI_Calc(HFI_t *hfi);   // 信号解调并进行位置观测
void HFI_Init(HFI_t *hfi, float ish, float ts, float kp, float ki, float fc, float offset);

#endif