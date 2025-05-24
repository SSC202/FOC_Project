#include "userinit.h"

/**
 * @brief   Userinit
 */
void user_init(void)
{
    system_dac_print = 0;
    /******************************************************
     * @brief   驱动电机FOC相关变量初始化
     */
    Slope_Init(&Drive_speed_slope, SYSTEM_SAMPLE_TIME, 0.1, 21);
    drive_speed_krise = 21; // 速度指令斜率50rpm/s;
    // alpha_beta 轴低通滤波器参数初始化
    LPF_Init(&Drive_ialpha_filter, 200, SYSTEM_SAMPLE_TIME);
    LPF_Init(&Drive_ibeta_filter, 200, SYSTEM_SAMPLE_TIME);
    HFI_Init(&Drive_hfi, SYSTEM_SAMPLE_TIME);
    /******************************************************
     * @brief   负载电机FOC相关变量初始化
     */
    Slope_Init(&Load_iq_slope, SYSTEM_SAMPLE_TIME, 0.01, 0.5);
}
