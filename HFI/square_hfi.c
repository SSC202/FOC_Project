#include "square_hfi.h"
#include "my_math.h"

/**
 * @brief   生成高频注入电压
 */
void HFI_Inject(HFI_t *hfi)
{
    // 计算电流注入角度对应的电压注入角度
    static float theta_inj;
    theta_inj = atanf((110 / 55) * tanf(hfi->theta_inj_i));
    while (theta_inj < -M_PI) {
        theta_inj = theta_inj + 2 * M_PI;
    }
    while (theta_inj > M_PI) {
        theta_inj = theta_inj - 2 * M_PI;
    }
    hfi->theta_inj = theta_inj;
    // 高频信噪比控制器
    static float id_sig, iq_sig;
    static float i_sh;
    iq_sig = (hfi->idq_h)[3].q - (hfi->idq_h)[1].q;
    id_sig = (hfi->idq_h)[3].d - (hfi->idq_h)[1].d;
    i_sh   = sqrt(iq_sig * iq_sig + id_sig * id_sig);

    static float u_bias; // 信噪比控制器输出的电压控制指令
    hfi->ish_pi_controller.ref = hfi->ish;
    hfi->ish_pi_controller.fdb = i_sh;
    if (hfi->enable == 0) {
        hfi->ish_pi_controller.ref       = 0;
        hfi->ish_pi_controller.fdb       = 0;
        hfi->ish_pi_controller.cur_error = 0;
        hfi->ish_pi_controller.output    = 0;
    }
    if (hfi->step == 2 || hfi->step == 4) {
        PID_Calc(&hfi->ish_pi_controller, hfi->enable, hfi->sample_time);
    }
    u_bias = hfi->ish_pi_controller.output;

    // 前馈补偿——高频阻抗曲线
    static float z_n, u_href;
    z_n      = (((0.0275f + 0.055f)) / 2.f) + (((0.055f - 0.0275f) / 2.f) * sinf(2 * (hfi->theta_inj_i) - (M_PI / 2)));
    u_href   = 5000 * z_n * hfi->ish; // 5000 为实测倍率(2.5kHz注入)
    hfi->u_h = u_bias + u_href;

    if (hfi->u_h > 90) {
        hfi->u_h = 90;
    }

    // 高频电压注入
    if (hfi->enable == 0) {
        hfi->step    = 1;
        hfi->udq_h.d = 0;
        hfi->udq_h.q = 0;
    } else {
        switch (hfi->step) {
            case 1:
                hfi->udq_h.d = hfi->u_h * cosf(hfi->theta_inj);
                hfi->udq_h.q = hfi->u_h * sinf(hfi->theta_inj);
                hfi->step    = 2;
                break;
            case 2:
                hfi->udq_h.d = hfi->u_h * cosf(hfi->theta_inj);
                hfi->udq_h.q = hfi->u_h * sinf(hfi->theta_inj);
                hfi->step    = 3;
                break;
            case 3:
                hfi->udq_h.d = -hfi->u_h * cosf(hfi->theta_inj);
                hfi->udq_h.q = -hfi->u_h * sinf(hfi->theta_inj);
                hfi->step    = 4;
                break;
            case 4:
                hfi->udq_h.d = -hfi->u_h * cosf(hfi->theta_inj);
                hfi->udq_h.q = -hfi->u_h * sinf(hfi->theta_inj);
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
    static float id_sig, iq_sig;
    // Update Current
    (hfi->idq_h[hfi->step - 1]).d = hfi->idqh_now.d;
    (hfi->idq_h[hfi->step - 1]).q = hfi->idqh_now.q;

    // demodulate
    if (hfi->step == 2 || hfi->step == 4) {
        iq_sig     = (hfi->idq_h)[3].q - (hfi->idq_h)[1].q;
        id_sig     = (hfi->idq_h)[3].d - (hfi->idq_h)[1].d;
        hfi->icomp = hfi->u_h * hfi->offset * sinf(2 * hfi->theta_inj);
        hfi->isig  = id_sig * sinf(hfi->theta_inj) + iq_sig * cosf(hfi->theta_inj) - hfi->icomp;
    }
}

/**
 * @brief   高频注入位置观测
 */
static void HFI_observe(HFI_t *hfi)
{
    // 不使能时采用有感运行数据
    if (hfi->enable == 0) {
        hfi->speed_obs = hfi->speed_true;
    }
    if (hfi->enable == 0) {
        hfi->theta_obs = hfi->theta_true;
    }
    if (hfi->enable == 0) {
        hfi->pll.ref               = 0;
        hfi->pll.fdb               = 0;
        hfi->pll.cur_error         = 0;
        hfi->pll.output            = 0;
        hfi->speed_lpf.input       = 0;
        hfi->speed_lpf.output      = 0;
        hfi->speed_lpf.output_last = 0;
    }

    // 使能时采用无感数据
    if (hfi->step == 2 || hfi->step == 4) {
        // PLL Caculate
        hfi->pll.ref = hfi->isig;
        hfi->pll.fdb = 0;
        PID_Calc(&hfi->pll, hfi->enable, hfi->sample_time);

        // Speed LPF
        hfi->speed_lpf.input = hfi->pll.output;
        LPF_Calc(&hfi->speed_lpf, hfi->enable);
        hfi->speed_obs = hfi->speed_lpf.output;

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
 * @param   ish     高频注入电流幅值
 * @param   ts      高频注入单次计算时间
 * @param   kp      PLL kp
 * @param   ki      PLL ki
 * @param   fc      Speed LPF fc
 */
void HFI_Init(HFI_t *hfi, float ish, float ts, float kp, float ki, float fc, float offset)
{
    hfi->ish         = ish;
    hfi->theta_inj_i = 0;
    hfi->sample_time = ts;
    hfi->offset      = offset; // 变轴系信号偏置

    PID_init(&hfi->pll, kp, ki, 0, INFINITY);

    LPF_Init(&hfi->speed_lpf, fc, hfi->sample_time);

    PID_init(&hfi->ish_pi_controller, 0.1, 1000, 0, 20);

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