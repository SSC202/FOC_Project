#include "square_hfi.h"

/**
 * @brief   生成高频注入电压
 */
void HFI_Inject(HFI_t *hfi)
{
    if (hfi->enable == 0) {
        hfi->step    = 1;
        hfi->udq_h.d = 0;
        hfi->udq_h.q = 0;
    } else {
        switch (hfi->step) {
            case 1:
                hfi->udq_h.d = hfi->u_h;
                hfi->udq_h.q = 0;
                hfi->step    = 2;
                break;
            case 2:
                hfi->udq_h.d = hfi->u_h;
                hfi->udq_h.q = 0;
                hfi->step    = 3;
                break;
            case 3:
                hfi->udq_h.d = -hfi->u_h;
                hfi->udq_h.q = 0;
                hfi->step    = 4;
                break;
            case 4:
                hfi->udq_h.d = -hfi->u_h;
                hfi->udq_h.q = 0;
                hfi->step    = 1;
                break;
            default:
                hfi->step    = 1;
                hfi->udq_h.d = 0;
                hfi->udq_h.q = 0;
                break;
        }
    }
}

/**
 * @brief   高频注入信号解调
 */
static void HFI_demodulate(HFI_t *hfi)
{
    // Update Current
    (hfi->idq_h[hfi->step - 1]).d = hfi->idqh_now.d;
    (hfi->idq_h[hfi->step - 1]).q = hfi->idqh_now.q;

    // demodulate
    if (hfi->step == 2 || hfi->step == 4) {
        hfi->isig = (hfi->idq_h)[3].q - (hfi->idq_h)[1].q;
    }
}

/**
 * @brief   高频注入位置观测
 */
static void HFI_observe(HFI_t *hfi)
{
    if (hfi->enable == 0) {
        hfi->speed_obs = hfi->speed_true;
    } else {
        hfi->speed_obs = hfi->speed_lpf.output;
    }
    if (hfi->enable == 0) {
        hfi->theta_obs = hfi->theta_true;
    }

    if (hfi->step == 2 || hfi->step == 4) {
        // PLL Caculate
        hfi->pll.ref = hfi->isig;
        hfi->pll.fdb = 0;
        PID_Calc(&hfi->pll, hfi->enable, hfi->sample_time);

        // Speed LPF
        hfi->speed_lpf.input = hfi->pll.output;
        LPF_Calc(&hfi->speed_lpf, hfi->enable);

        // Theta
        hfi->theta_obs += hfi->pll.output * hfi->sample_time;

        while (hfi->theta_obs < -M_PI) {
            hfi->theta_obs = hfi->theta_obs + 2 * M_PI;
        }
        while (hfi->theta_obs > M_PI) {
            hfi->theta_obs = hfi->theta_obs - 2 * M_PI;
        }
    }
}

/**
 * @brief   高频注入一次计算
 */
void HFI_Calc(HFI_t *hfi)
{
    // 信号解调
    HFI_demodulate(hfi);
    // 位置观测
    HFI_observe(hfi);
}

/**
 * @brief   高频注入参数初始化
 * @param   hfi     高频注入结构体
 * @param   uh      高频注入电压幅值
 * @param   ts      高频注入单次计算时间
 * @param   kp      PLL kp
 * @param   ki      PLL ki
 * @param   fc      Speed LPF fc
 */
void HFI_Init(HFI_t *hfi, float uh, float ts, float kp, float ki, float fc)
{
    hfi->u_h         = uh;
    hfi->sample_time = ts;

    PID_init(&hfi->pll, kp, ki, 0, INFINITY);

    LPF_Init(&hfi->speed_lpf, fc, hfi->sample_time);

    hfi->enable = 0;
    hfi->step   = 1;

    hfi->idq_h[0].d = 0;
    hfi->idq_h[0].q = 0;
    hfi->idq_h[1].d = 0;
    hfi->idq_h[1].q = 0;
    hfi->idq_h[2].d = 0;
    hfi->idq_h[2].q = 0;
    hfi->idq_h[3].d = 0;
    hfi->idq_h[3].q = 0;
}