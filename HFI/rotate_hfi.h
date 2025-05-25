#ifndef __ROTATE_HFI_H
#define __ROTATE_HFI_H

#include "my_math.h"
#include "coordinate_transform.h"

typedef struct
{
    // Init Parameters 初始化相关参数
    float f_inj;    // 注入频率
    float vf_ratio; // 注入恒压频比比值

    float t_sample; // 采样时间

    PID_t pll;       // 锁相环
    LPF_t speed_lpf; // 速度低通滤波器

    uint8_t filter_type_1; // HPF1(z) type
    uint8_t filter_type_2; // HPF2(z) type
    float HPF1_fc_ratio;         // HPF1(z) 截止频率比值
    float HPF2_fc_ratio;         // HPF2(z) 截止频率比值
    float HPF1_fc;         // HPF1(z) 截止频率
    float HPF2_fc;         // HPF2(z) 截止频率
    HPF_t HPF1_alpha;      // alpha 轴 HPF1(z)
    HPF_t HPF1_beta;       // beta 轴 HPF1(z)
    HPF_t HPF2_alpha;      // alpha 轴 HPF1(z)
    HPF_t HPF2_beta;       // beta 轴 HPF1(z)

    // Control Parameters 运行控制相关参数
    uint8_t enable;            // HFI 使能
    uint8_t polar_comp_enable; // 极性补偿使能

    // Output Parameters 输出参数
    alpha_beta_t u_alpha_beta_h; // 注入电压
    float theta_obs;             // 估计角度
    float speed_obs;             // 估计速度
    float theta_error;           // 角度估计误差
    float speed_error;           // 转速估计误差

    // Input Parameters 输入参数
    alpha_beta_t i_alpha_beta; // 输入响应电流(原始值)
    float theta_true;          // 真实角度
    float speed_true;          // 真实速度

    // Temporary variables 临时变量
    float phase_h; // 注入高频旋转电压信号相位
    float u_h;     // 注入电压信号幅值

    alpha_beta_t i_alpha_beta_h;   // 输入高频响应电流
    dq_t i_dhqh_h;                 // 高频响应电流变换到以 2*pi*f_inj 正向旋转的 dh 轴上
    dq_t i_dhqh_n_h;               // i_dhqh_h 经 HPF 后得到的负序分量
    alpha_beta_t i_alpha_beta_n_h; // 同步轴系高通滤波器输出的 alpha_beta 轴负序电流
    float error_dem;               // 误差信号 f(err)
} HFI_t;

void HFI_Inject(HFI_t *hfi);
void HFI_Calc(HFI_t *hfi);
void HFI_Init(HFI_t *hfi, float tsample);

#endif