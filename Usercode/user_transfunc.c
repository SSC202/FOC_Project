#include "user_transfunc.h"

/****************************** PID 控制器 ********************************* */

/**
 * @brief   增量式 PID 控制器初始化
 * @param   pid         PID 控制器句柄
 * @param   kp          Kp  比例参数
 * @param   ki          Ki  积分参数(考虑采样时间)
 * @param   kd          Kd  微分参数
 * @param   outputMax   输出饱和值
 */
void PID_init(PID_t *pid, float kp, float ki, float kd, float outputMax)
{
    pid->KP        = kp;
    pid->KI        = ki;
    pid->KD        = kd;
    pid->outputMax = outputMax;
}

/**
 * @brief   增量式PID计算
 * @param   pid         PID 控制器句柄
 * @param   enable      使能信号    若此值为0,PID控制器的输入,反馈和输出都置为0
 * @param   t_sample    采样时间
 */
void PID_Calc(PID_t *pid, uint8_t enable, float t_sample)
{
    if (enable == 1) {
        pid->cur_error = pid->ref - pid->fdb;
        pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * t_sample * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
        pid->error[0] = pid->error[1];
        pid->error[1] = pid->ref - pid->fdb;
        /*设定输出上限*/
        if (pid->output > pid->outputMax) pid->output = pid->outputMax;
        if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;
    } else {
        pid->ref       = 0;
        pid->fdb       = 0;
        pid->output    = 0;
        pid->cur_error = 0;
        pid->error[0]  = 0;
        pid->error[1]  = 0;
    }
}

/****************************** IIR 滤波器 ********************************* */

/**
 * @brief   数字一阶低通滤波器初始化(后向差分法)
 * @param   lpf         滤波器句柄
 * @param   f_c         截止频率
 * @param   t_sample    采样时间
 */
void LPF_Init(LPF_t *lpf, float f_c, float t_sample)
{
    lpf->wc          = 2 * M_PI * f_c;
    lpf->tsample     = t_sample;
    lpf->alpha       = (lpf->wc * lpf->tsample) / ((lpf->wc * lpf->tsample) + 1.0f);
    lpf->output_last = 0;
    lpf->output      = 0;
    lpf->input       = 0;
}

/**
 * @brief   数字一阶低通滤波器计算(后向差分法)
 * @param   lpf         滤波器句柄
 * @param   enable      滤波器使能参数
 */
void LPF_Calc(LPF_t *lpf, uint8_t enable)
{
    if (enable == 1) {
        lpf->output      = lpf->output_last + lpf->alpha * (lpf->input - lpf->output_last);
        lpf->output_last = lpf->output;
    } else {
        lpf->output      = 0;
        lpf->output_last = 0;
        lpf->input       = 0;
    }
}

/**
 * @brief   巴特沃斯高通滤波器参数设计
 * @param   hpf         滤波器句柄
 * @param   f_c         截止频率
 * @param   type        滤波器类型
 * @note    HPF(z)通用表达式：y[n] = a1 * y[n-1] + a2 * y[n-2] + b0 * x[n] + b1 * x[n-1] + b2 * x[n-2]
 * @note    *************************************************************************
 * @note    type = 1   一阶巴特沃斯HPF(z)，BackwardEuler法
 * @note    a1 = 1 / (1 + wc * Ts); a2 = 0; b0 = -b1 = 1 / (1 + wc * Ts); b2 = 0
 * @note    *************************************************************************
 * @note    type = 2   二阶巴特沃斯HPF(z)，BackwardEuler法
 * @note    a1 = (1.414 * wc * Ts + 2) / (wc * wc * Ts * Ts + 1.414 * wc * Ts + 1);
 * @note    a2 = - 1 / (wc * wc * Ts * Ts + 1.414 * wc * Ts + 1); b0 = b2 = - a2; b1 = 2 *  a2
 * @note    *************************************************************************
 * @note    type = 3   一阶巴特沃斯HPF(z)，Tustin法
 * @note    a1 = (2 - wc * Ts) / (2 + wc * Ts); a2 = 0; b0 = -b1 = 2 / (2 + wc * Ts); b2 = 0
 * @note    *************************************************************************
 * @note    type = 4   二阶巴特沃斯HPF(z)，Tustin法
 * @note    a1 = (8 - 2 * wc * wc * Ts * Ts) / (wc * wc * Ts * Ts + 2.828 * wc * Ts + 4);
 * @note    a2 = -(wc * wc * Ts * Ts - 2.828 * wc * Ts + 4) / (wc * wc * Ts * Ts + 2.828 * wc * Ts + 4)
 * @note    b0 = b2 = 4 / (wc * wc * Ts * Ts + 2.828 * wc * Ts + 4); b1 = -2 * b0
 *
 */
void HPF_Design(HPF_t *hpf, float f_c, uint8_t type, float t_sample)
{
    hpf->wc      = 2 * M_PI * f_c;
    hpf->tsample = t_sample;
    hpf->type    = type;

    switch (hpf->type) {
        // type1
        case 1:
            hpf->a1 = 1.0f / ((hpf->wc * hpf->tsample) + 1.0f);
            hpf->a2 = 0;
            hpf->b0 = 1.0f / ((hpf->wc * hpf->tsample) + 1.0f);
            hpf->b1 = -hpf->b0;
            hpf->b2 = 0;
            break;
        // type 2
        case 2:
            hpf->a1 = ((1.414f * hpf->wc * hpf->tsample) + 2.0f) / ((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) + (1.414f * hpf->wc * hpf->tsample) + 1.0f);

            hpf->a2 = -1.0f / ((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) + (1.414f * hpf->wc * hpf->tsample) + 1.0f);
            hpf->b0 = 1.0f / ((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) + (1.414f * hpf->wc * hpf->tsample) + 1.0f);
            hpf->b1 = -2.0f * hpf->b0;
            hpf->b2 = hpf->b0;
            break;
        // type 3
        case 3:
            hpf->a1 = (2.0f - (hpf->wc * hpf->tsample)) / ((hpf->wc * hpf->tsample) + 2.0f);
            hpf->a2 = 0;
            hpf->b0 = 2.0f / ((hpf->wc * hpf->tsample) + 2.0f);
            hpf->b1 = -hpf->b0;
            hpf->b2 = 0;
            break;
        // type 4
        case 4:
            hpf->a1 = (8.0f - 2.0f * hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) / ((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) + (2.828f * hpf->wc * hpf->tsample) + 4.0f);

            hpf->a2 = -((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) - (2.828f * hpf->wc * hpf->tsample) + 4.0f) / ((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) + (2.828f * hpf->wc * hpf->tsample) + 4.0f);

            hpf->b0 = 4.0f / ((hpf->wc * hpf->wc * hpf->tsample * hpf->tsample) + (2.828f * hpf->wc * hpf->tsample) + 4.0f);
            hpf->b1 = -2.0f * hpf->b0;
            hpf->b2 = hpf->b0;
            break;
        default:
            // 默认： 输出等于输入
            hpf->a1 = 0;
            hpf->a2 = 0;
            hpf->b0 = 1.0f;
            hpf->b1 = 0;
            hpf->b2 = 0;
            break;
    }
}

/**
 * @brief   高通滤波器初始化
 * @param   hpf         滤波器句柄
 * @param   t_sample    采样时间
 * @param   type        滤波器类型
 * @note    巴特沃斯通用表达式：y[n] = a1 * y[n-1] + a2 * y[n-2] + b0 * x[n] + b1 * x[n-1] + b2 * x[n-2]
 */
void HPF_Init(HPF_t *hpf, float t_sample, float f_c, uint8_t type)
{
    hpf->tsample = t_sample;
    hpf->wc      = 2 * M_PI * f_c;

    // 滤波器参数初始化
    HPF_Design(hpf, f_c, type, hpf->tsample);

    // 输入输出参数初始化
    hpf->input_last[0]  = 0;
    hpf->input_last[1]  = 0;
    hpf->input          = 0;
    hpf->output_last[0] = 0;
    hpf->output_last[1] = 0;
    hpf->output         = 0;
}

/**
 * @brief   巴特沃斯高通滤波器计算
 * @param   hpf         滤波器句柄
 * @param   enable      滤波器使能参数
 */
void HPF_Calc(HPF_t *hpf, uint8_t enable)
{
    if (enable == 1) {
        hpf->output         = hpf->a1 * hpf->output_last[1] + hpf->a2 * hpf->output_last[0] + hpf->b0 * hpf->input + hpf->b1 * hpf->input_last[1] + hpf->b2 * hpf->input_last[0];
        hpf->input_last[0]  = hpf->input_last[1];
        hpf->input_last[1]  = hpf->input;
        hpf->output_last[0] = hpf->output_last[1];
        hpf->output_last[1] = hpf->output;
    } else {
        hpf->output         = 0;
        hpf->input_last[0]  = 0;
        hpf->input_last[1]  = 0;
        hpf->output_last[0] = 0;
        hpf->output_last[1] = 0;
    }
}

/****************************** 指令生成器 ********************************* */

/**
 * @brief   生成斜坡指令（转速/负载转矩）
 * @param   slope         斜坡指令句柄
 */
void Slope_Module(Slope_t *slope, float t_sample, uint8_t enable)
{
    slope->tsample = t_sample;
    if ((slope->ref_in - slope->ref_out > slope->gap) || (slope->ref_in - slope->ref_out == slope->gap)) {
        slope->ref_out = slope->ref_out + slope->K_rise * t_sample;
    }
    if ((slope->ref_in - slope->ref_out < slope->gap) || (slope->ref_in - slope->ref_out == slope->gap)) {
        slope->ref_out = slope->ref_out - slope->K_rise * t_sample;
    }
    slope->ref_out = enable * slope->ref_out;
}

/**
 * @brief   初始化斜坡指令
 * @param   slope         斜坡指令句柄
 * @param   t_sample      采样时间
 * @param   gap           间隙
 * @param   krise         斜坡指令斜率
 */
void Slope_Init(Slope_t *slope, float t_sample, float gap, float krise)
{
    slope->tsample = t_sample;
    slope->gap     = gap;
    slope->K_rise  = krise;
    slope->ref_in  = 0;
    slope->ref_out = 0;
}