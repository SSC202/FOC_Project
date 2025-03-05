#include "my_math.h"

/**
 * @brief   机械角度转换为电角度函数并归一化
 */
float normalize(int pole_pairs, float mechine_angle, float offset)
{
    float electric_angle = pole_pairs * mechine_angle - offset;
    float out            = electric_angle;
    while (out < -M_PI) {
        out = out + 2 * M_PI;
    }
    while (out > M_PI) {
        out = out - 2 * M_PI;
    }
    return out;
}

/**
 * @brief   取最大值函数
 */
float get_max(float a, float b, float c)
{
    float ret = 0;
    if (a >= b && a >= c)
        ret = a;
    else if (b >= a && b >= c)
        ret = b;
    else
        ret = c;
    return ret;
}

/**
 * @brief   取最小值函数
 */
float get_min(float a, float b, float c)
{
    float ret = 0;
    if (a <= b && a <= c)
        ret = a;
    else if (b <= a && b <= c)
        ret = b;
    else
        ret = c;
    return ret;
}

/**
 * @brief   取中间值函数
 */
float get_middle(float a, float b, float c)
{
    float ret = 0;
    ret       = a + b + c - get_max(a, b, c) - get_min(a, b, c);
    return ret;
}

/****************************** PID 控制器 ********************************* */

/**
 * @brief   通用 PID 控制器初始化
 * @param   pid         PID 控制器句柄
 * @param   kp          Kp
 * @param   ki          Ki
 * @param   kd          Kd
 * @param   outputMax   最大输出值
 * @attention   PID 为增量式
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
 */
void PID_Calc(PID_t *pid, uint8_t enable, float t_sample)
{
    pid->ref       = pid->ref * enable;
    pid->fdb       = pid->fdb * enable;
    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * t_sample * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->output   = pid->output * enable;
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->ref - pid->fdb;
    /*设定输出上限*/
    if (pid->output > pid->outputMax) pid->output = pid->outputMax;
    if (pid->output < -pid->outputMax) pid->output = -pid->outputMax;
}