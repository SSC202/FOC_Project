#include "dq_square_hfi.h"

/**
 * @brief   增量式PID计算(PLL计算)
 * @param   pll         PLL 控制器句柄
 * @param   enable      使能信号    若此值为0,PID控制器的输入,反馈和输出都置为0
 * @param   t_sample    采样时间
 */
static void PLL_Calc(PLL_t *pll, uint8_t enable, float t_sample)
{
    pll->cur_error = pll->cur_error * enable;
    pll->output += pll->KP * (pll->cur_error - pll->error[1]) + pll->KI * t_sample * pll->cur_error;
    pll->output   = pll->output * enable;
    pll->error[0] = pll->error[1];
    pll->error[1] = pll->cur_error;
}

/**
 * @brief   HFI 观测器初始化
 * @param   uh              注入电压幅值
 * @param   theta_inj       注入电压方向(电角度,取弧度制)
 * @param   t_sample        系统采样时间
 * @param   Ld              d 轴电感
 * @param   Lq              q 轴电感
 * @param   pll_kp          锁相环 Kp
 * @param   pll_ki          锁相环 Ki
 * @param   theta_obs_init  电角度初值
 */
void observer_Init(hfi_t *hfi, float uh, float theta_inj, float t_sample, float Ld, float Lq, double pll_kp, double pll_ki)
{
    hfi->u_h                = uh;
    hfi->electric_theta_inj = theta_inj;
    hfi->clap               = 1; // 初始化为第一拍
    hfi->inject_counter     = 1; // 第一拍为正
    hfi->hfi_inject_t       = 2 * t_sample;
    hfi->Ld                 = Ld;
    hfi->Lq                 = Lq;

    hfi->pll.cur_error = 0;
    hfi->pll.error[0]  = 0;
    hfi->pll.error[1]  = 0;
    hfi->pll.output    = 0;
    hfi->pll.KI        = pll_ki;
    hfi->pll.KP        = pll_kp;

    hfi->electric_theta_obs = 0;
    hfi->speed_obs          = 0;

    hfi->delta_id = 0;
    hfi->delta_iq = 0;

    hfi->id_sig = 0;
    hfi->iq_sig = 0;
    hfi->i_sig  = 0;
    hfi->i_comp = 0;
    hfi->i_err  = 0;
}

/**
 * @brief   HFI 观测器计算
 */
void observer_Calc(hfi_t *hfi, uint8_t enable)
{
    // 信号解调
    hfi->id_sig = hfi->delta_id * (float)hfi->inject_counter;
    hfi->iq_sig = hfi->delta_iq * (float)hfi->inject_counter;

    hfi->i_sig  = hfi->id_sig * sinf(hfi->electric_theta_inj) + hfi->iq_sig * cosf(hfi->electric_theta_inj);
    hfi->i_comp = (hfi->u_h * hfi->hfi_inject_t * (hfi->Ld + hfi->Lq) * sinf(2 * hfi->electric_theta_inj)) / (hfi->Ld * hfi->Lq);
    hfi->i_err  = hfi->i_sig - hfi->i_comp;

    // 锁相环计算
    hfi->pll.cur_error = hfi->i_err;
    PLL_Calc(&(hfi->pll), enable, hfi->hfi_inject_t);
    hfi->speed_obs = hfi->pll.output;
    hfi->electric_theta_obs += hfi->pll.output * hfi->hfi_inject_t;

    while (hfi->electric_theta_obs < -M_PI) {
        hfi->electric_theta_obs = hfi->electric_theta_obs + 2 * M_PI;
    }
    while (hfi->electric_theta_obs > M_PI) {
        hfi->electric_theta_obs = hfi->electric_theta_obs - 2 * M_PI;
    }
}