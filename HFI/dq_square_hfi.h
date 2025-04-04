/**
 * @file    脉振方波注入相关代码
 * @note    采用脉振方波变轴系注入,dq轴均有同相的注入电压,输入角度设置为0时为d轴注入
 */
#ifndef __DQ_SQUARE_HFI_H
#define __DQ_SQUARE_HFI_H

#include "my_math.h"

/**
 * PLL 相关定义
 */
typedef struct {
    float KP;        // 参数P
    float KI;        // 参数I
    float cur_error; // 当前误差
    float error[2];  // 前两次误差
    float output;    // 输出值
} PLL_t;

typedef struct
{
    // HFI 注入信号生成模块
    float u_h;                // 注入电压信号幅值
    float electric_theta_inj; // 注入电压信号角度
    uint8_t clap;             // 注入节拍(4拍,1,2,3,4)
    int inject_counter;       // 注入电压符号
    float hfi_inject_t;       // 注入方波高电平时间

    // HFI 观测器
    float electric_theta_obs; // 观测电角度
    float speed_obs;          // 观测速度

    float idh; // 高频电流 d 轴
    float iqh; // 高频电流 q 轴
    float idh_last;
    float iqh_last;

    float delta_id;
    float delta_iq;

    float id_sig;
    float iq_sig;
    float i_sig;
    float i_comp;
    float i_err;

    float offset; // 变轴系注入偏置量增益

    PLL_t pll; // 锁相环(增量式PID)

    LPF_t speed_lpf; // 速度低通滤波器

} hfi_t;

void observer_Calc(hfi_t *hfi, uint8_t enable);
void observer_Init(hfi_t *hfi, float uh, float theta_inj, float t_sample, float offset, double pll_kp, double pll_ki);

#endif