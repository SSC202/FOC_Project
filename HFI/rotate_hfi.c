#include "rotate_hfi.h"

/**
 * @brief   高频注入参数初始化
 */
void HFI_Init(HFI_t *hfi, float tsample)
{
    hfi->enable        = 0;    // 不使能高频注入
    hfi->f_inj         = 500;  // 注入频率 500 Hz
    hfi->vf_ratio      = 0.01; // 恒压频比
    hfi->filter_type_1 = 4;    // 默认使用二阶 Tustin 法离散化HPF(z)
    hfi->filter_type_2 = 4;
    hfi->HPF1_fc_ratio       = 0.1; // 默认截止频率
    hfi->HPF2_fc_ratio       = 0.2;
    hfi->t_sample      = tsample;

    PID_init(&hfi->pll, 4000, 2000, 200, INFINITY);

    hfi->polar_comp_enable = 0; // 不使用极性补偿

    HPF_Init(&(hfi->HPF1_alpha), hfi->t_sample, hfi->HPF1_fc, hfi->filter_type_1);
    HPF_Init(&(hfi->HPF1_beta), hfi->t_sample, hfi->HPF1_fc, hfi->filter_type_1);
    HPF_Init(&(hfi->HPF2_alpha), hfi->t_sample, hfi->HPF2_fc, hfi->filter_type_2);
    HPF_Init(&(hfi->HPF2_beta), hfi->t_sample, hfi->HPF2_fc, hfi->filter_type_2);

    LPF_Init(&(hfi->speed_lpf), 1, hfi->t_sample);
    
}

/**
 * @brief   生成高频注入电压
 * @param   hfi 高频注入结构体
 * @note    同时生成当前的高频信号相位,phase_h = n * whTs
 * @note    采用恒压频比注入,u_h = vf_ratio * f_inj
 */
void HFI_Inject(HFI_t *hfi)
{
    // phase generate
    if (hfi->enable == 0) {
        hfi->phase_h = 0.0f;
    } else {
        hfi->phase_h += (2.0f * M_PI * hfi->f_inj * hfi->t_sample); // 高频旋转电压矢量相位每一拍增加wh*Ts
    }

    // normalize phase_h
    if (hfi->phase_h > 2 * M_PI) {
        hfi->phase_h -= 2 * M_PI;
    } else if (hfi->phase_h < -2 * M_PI) {
        hfi->phase_h += 2 * M_PI;
    }

    // vf control
    if (0 < hfi->f_inj && hfi->f_inj < 5000) {
        hfi->u_h = hfi->vf_ratio * hfi->f_inj;
    } else if (5000 < hfi->f_inj && hfi->f_inj < 10000) {
        hfi->u_h = hfi->vf_ratio * (10000 - hfi->f_inj);
    } else if (-5000 < hfi->f_inj && hfi->f_inj < 0) {
        hfi->u_h = -hfi->vf_ratio * hfi->f_inj;
    } else if (-10000 < hfi->f_inj && hfi->f_inj < -5000) {
        hfi->u_h = hfi->vf_ratio * (10000 + hfi->f_inj);
    }

    // generate inject voltage
    if (hfi->enable == 1) {
        hfi->u_alpha_beta_h.alpha = hfi->u_h * cosf(hfi->phase_h);
        hfi->u_alpha_beta_h.beta  = hfi->u_h * sinf(hfi->phase_h);
    } else {
        hfi->u_alpha_beta_h.alpha = 0;
        hfi->u_alpha_beta_h.beta  = 0;
    }
}

/**
 * @brief   高频注入一次计算
 */
void HFI_Calc(HFI_t *hfi)
{
    /**********************************************
     * @brief   step 1: HPF Design
     */
    // HPF(z) 截止频率及 1.5Ts 延迟补偿角计算
    float comp;
    if (0 < hfi->f_inj && hfi->f_inj < 5000) {
        hfi->HPF1_fc = hfi->HPF1_fc_ratio * hfi->f_inj;
        hfi->HPF2_fc = hfi->HPF2_fc_ratio * hfi->f_inj;
        comp         = 2 * M_PI * hfi->f_inj * hfi->t_sample * 0.5f;
    } else if (5000 < hfi->f_inj && hfi->f_inj < 10000) {
        hfi->HPF1_fc = hfi->HPF1_fc_ratio * (10000 - hfi->f_inj);
        hfi->HPF2_fc = hfi->HPF2_fc_ratio * (10000 - hfi->f_inj);
        comp         = 2 * M_PI * (hfi->f_inj - 10000) * hfi->t_sample * 0.5f;
    } else if (-5000 < hfi->f_inj && hfi->f_inj < 0) {
        hfi->HPF1_fc = -hfi->HPF1_fc_ratio * hfi->f_inj;
        hfi->HPF2_fc = -hfi->HPF2_fc_ratio * hfi->f_inj;
        comp         = 2 * M_PI * hfi->f_inj * hfi->t_sample * 0.5f;
    } else if (-10000 < hfi->f_inj && hfi->f_inj < -5000) {
        hfi->HPF1_fc = hfi->HPF1_fc_ratio * (10000 + hfi->f_inj);
        hfi->HPF2_fc = hfi->HPF2_fc_ratio * (10000 + hfi->f_inj);
        comp         = 2 * M_PI * (hfi->f_inj + 10000) * hfi->t_sample * 0.5f;
    }

    // 滤波器参数设计，根据 filter type 和 HPF(z)1/HPF(z)2 截止设计两个 HPF(z) 的参数
    HPF_Design(&(hfi->HPF1_alpha), hfi->HPF1_fc, hfi->filter_type_1, hfi->t_sample);
    HPF_Design(&(hfi->HPF1_beta), hfi->HPF1_fc, hfi->filter_type_1, hfi->t_sample);
    HPF_Design(&(hfi->HPF2_alpha), hfi->HPF2_fc, hfi->filter_type_2, hfi->t_sample);
    HPF_Design(&(hfi->HPF2_beta), hfi->HPF2_fc, hfi->filter_type_2, hfi->t_sample);

    /**********************************************
     * @brief   step 2: Signal demodulate
     */
    // 提取 alpha-beta 轴高频响应电流
    hfi->HPF1_alpha.input = hfi->i_alpha_beta.alpha;
    hfi->HPF1_beta.input  = hfi->i_alpha_beta.beta;
    HPF_Calc(&(hfi->HPF1_alpha), hfi->enable);
    HPF_Calc(&(hfi->HPF1_beta), hfi->enable);
    hfi->i_alpha_beta_h.alpha = hfi->HPF1_alpha.output;
    hfi->i_alpha_beta_h.beta  = hfi->HPF1_beta.output;

    // 变换到 dh_qh 同步旋转坐标系上
    alphabeta_2_dq(&(hfi->i_alpha_beta_h), &(hfi->i_dhqh_h), hfi->phase_h);

    // HPF(z)2，滤除正序分量
    hfi->HPF2_alpha.input = hfi->i_dhqh_h.d;
    hfi->HPF2_beta.input  = hfi->i_dhqh_h.q;
    HPF_Calc(&(hfi->HPF2_alpha), hfi->enable);
    HPF_Calc(&(hfi->HPF2_beta), hfi->enable);
    hfi->i_dhqh_n_h.d = hfi->HPF2_alpha.output;
    hfi->i_dhqh_n_h.q = hfi->HPF2_beta.output;

    // 变换回 alpha-beta 坐标系上
    dq_2_alphabeta(&(hfi->i_dhqh_n_h), &(hfi->i_alpha_beta_n_h), hfi->phase_h);

    // 外差法计算误差信号f(err)
    if (0 < hfi->f_inj && hfi->f_inj < 5000) {
        hfi->error_dem = (-hfi->i_alpha_beta_n_h.alpha * cosf(2 * hfi->theta_obs - hfi->phase_h + comp) - hfi->i_alpha_beta_n_h.beta * sinf(2 * hfi->theta_obs - hfi->phase_h + comp));
    } else if (5000 < hfi->f_inj && hfi->f_inj < 10000) {
        hfi->error_dem = (+hfi->i_alpha_beta_n_h.alpha * cosf(2 * hfi->theta_obs - hfi->phase_h + comp) + hfi->i_alpha_beta_n_h.beta * sinf(2 * hfi->theta_obs - hfi->phase_h + comp));
    } else if (-5000 < hfi->f_inj && hfi->f_inj < 0) {
        hfi->error_dem = (+hfi->i_alpha_beta_n_h.alpha * cosf(2 * hfi->theta_obs - hfi->phase_h + comp) + hfi->i_alpha_beta_n_h.beta * sinf(2 * hfi->theta_obs - hfi->phase_h + comp));
    } else if (-10000 < hfi->f_inj && hfi->f_inj < -5000) {
        hfi->error_dem = (-hfi->i_alpha_beta_n_h.alpha * cosf(2 * hfi->theta_obs - hfi->phase_h + comp) - hfi->i_alpha_beta_n_h.beta * sinf(2 * hfi->theta_obs - hfi->phase_h + comp));
    }

    /**********************************************
     * @brief   step 3: pll observer
     */
    hfi->pll.ref = hfi->error_dem;
    hfi->pll.fdb = 0;
    PID_Calc(&(hfi->pll), hfi->enable, hfi->t_sample);

    // Speed LPF
    hfi->speed_lpf.input = hfi->pll.output;
    LPF_Calc(&hfi->speed_lpf, hfi->enable);
    hfi->speed_obs = hfi->speed_lpf.output;

    // Theta
    hfi->theta_obs += hfi->pll.output * hfi->t_sample;

    while (hfi->theta_obs < -M_PI) {
        hfi->theta_obs = hfi->theta_obs + 2 * M_PI;
    }
    while (hfi->theta_obs > M_PI) {
        hfi->theta_obs = hfi->theta_obs - 2 * M_PI;
    }

    hfi->theta_obs = normalize(1, hfi->theta_obs, 0);
    if (hfi->polar_comp_enable == 1) {
        hfi->theta_obs = hfi->theta_obs + M_PI;
    }
    hfi->theta_obs = normalize(1, hfi->theta_obs, 0);

    // 不使能时采用有感运行数据
    if (hfi->enable == 0) {
        hfi->theta_obs = hfi->theta_true;
        hfi->speed_obs = hfi->speed_true;
    }

    // 误差分析
    hfi->theta_error = hfi->theta_obs - hfi->theta_true;
    hfi->theta_error = normalize(1, hfi->theta_error, 0); // 角度观测误差归一化
    hfi->speed_error = hfi->speed_obs - hfi->speed_true;
}