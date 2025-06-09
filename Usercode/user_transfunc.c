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


