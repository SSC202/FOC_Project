#include "drivefoc.h"

/**
 * @brief   Drive Motor FOC Control caculate, use it in interrupt
 */
void drive_foc_calc(void)
{
    /************************************************************
     * @brief   HFI Caculate
     */
    // current ABC-to-alphabeta
    abc_2_alphabeta(&Drive_iabc, &Drive_ialphabeta);
    alphabeta_2_dq(&Drive_ialphabeta, &Drive_idq, Drive_AD2S.Electrical_Angle);

    // HFI Calculate
    Drive_hfi.theta_true = Drive_AD2S.Electrical_Angle;
    Drive_hfi.speed_true = Drive_AD2S.Speed;
    HFI_Calc(&Drive_hfi);

    /**
     * @brief       信噪比实验测试用代码（计算高频电流幅值）
     * @note        先分别计算 alpha 轴和 beta 轴负序电流幅值，再取平均值
     */
    // i_alpha_nh_1 = Drive_hfi.i_alpha_beta_n_h.alpha * sinf(- Drive_hfi.phase_h);
    // i_alpha_nh_2 = Drive_hfi.i_alpha_beta_n_h.alpha * cosf(- Drive_hfi.phase_h);
    // i_alpha_nh_1_filter.input = i_alpha_nh_1;
    // i_alpha_nh_2_filter.input = i_alpha_nh_2;
    // LPF_Calc(&i_alpha_nh_1_filter);
    // LPF_Calc(&i_alpha_nh_2_filter);
    // i_alpha_nh_3 = i_alpha_nh_1_filter.output;
    // i_alpha_nh_4 = i_alpha_nh_2_filter.output;
    // Inh_1 = fast_sqrt((2 * i_alpha_nh_3) * (2 * i_alpha_nh_3) + (2 * i_alpha_nh_4) * (2 * i_alpha_nh_4)) ;

    // i_beta_nh_1 = Drive_hfi.i_alpha_beta_n_h.beta * sinf(- Drive_hfi.phase_h);
    // i_beta_nh_2 = Drive_hfi.i_alpha_beta_n_h.beta * cosf(- Drive_hfi.phase_h);
    // i_beta_nh_1_filter.input = i_beta_nh_1;
    // i_beta_nh_2_filter.input = i_beta_nh_2;
    // LPF_Calc(&i_beta_nh_1_filter);
    // LPF_Calc(&i_beta_nh_2_filter);
    // i_beta_nh_3 = i_beta_nh_1_filter.output;
    // i_beta_nh_4 = i_beta_nh_2_filter.output;
    // Inh_2 = fast_sqrt((2 * i_beta_nh_3) * (2 * i_beta_nh_3) + (2 * i_beta_nh_4) * (2 * i_beta_nh_4)) ;

    // Inh = (Inh_1 + Inh_2) * 0.5f;
    /************************************************************
     * @brief   FOC Caculate
     */

    // Speed loop
    Drive_speed_pi.ref = drive_speed_ref;
    // Drive_speed_pi.fdb = Drive_AD2S.Speed;
    Drive_speed_pi.fdb = Drive_hfi.speed_obs;
    PID_Calc(&Drive_speed_pi, system_enable, SYSTEM_SAMPLE_TIME);

    // Current loop
    // current LPF
    Drive_ialpha_filter.input = Drive_ialphabeta.alpha;
    Drive_ibeta_filter.input  = Drive_ialphabeta.beta;
    LPF_Calc(&Drive_ialpha_filter, system_enable);
    LPF_Calc(&Drive_ibeta_filter, system_enable);
    Drive_ialphabetal.alpha = Drive_ialpha_filter.output;
    Drive_ialphabetal.beta  = Drive_ibeta_filter.output;

    // current alphabeta-to-dq
    // alphabeta_2_dq(&Drive_ialphabetal, &Drive_idql, Drive_AD2S.Electrical_Angle);
    alphabeta_2_dq(&Drive_ialphabetal, &Drive_idql, Drive_hfi.theta_obs);

    // (id=0 control)Current PI Controller
    // d-axis
    Drive_id_pi.ref = 0;
    Drive_id_pi.fdb = Drive_idql.d;
    PID_Calc(&Drive_id_pi, system_enable, SYSTEM_SAMPLE_TIME);
    Drive_udql.d = Drive_id_pi.output;

    // q-axis
    Drive_iq_pi.ref = Drive_speed_pi.output;
    Drive_iq_pi.fdb = Drive_idql.q;
    PID_Calc(&Drive_iq_pi, system_enable, SYSTEM_SAMPLE_TIME);
    Drive_udql.q = Drive_iq_pi.output;

    // dq_2_abc(&Drive_udql, &Drive_uabcl, Drive_AD2S.Electrical_Angle);
    dq_2_abc(&Drive_udql, &Drive_uabcl, Drive_hfi.theta_obs);

    /************************************************************
     * @brief   HFI Inject
     */
    HFI_Inject(&Drive_hfi);
    Drive_ualphabetah.alpha = Drive_hfi.u_alpha_beta_h.alpha;
    Drive_ualphabetah.beta  = Drive_hfi.u_alpha_beta_h.beta;
    alphabeta_2_abc(&Drive_ualphabetah, &Drive_uabch);

    /************************************************************
     * @brief   SVPWM
     */
    Drive_uabc.a = Drive_uabcl.a + Drive_uabch.a;
    Drive_uabc.b = Drive_uabcl.b + Drive_uabch.b;
    Drive_uabc.c = Drive_uabcl.c + Drive_uabch.c;
    e_svpwm(&Drive_uabc, 201, &Drive_duty_abc);

    /************************************************************
     * @brief   DAC Output
     * @note    0 : 输出0、2048、4095对应模拟量用于校正示波器偏置
     * @note    1 : 输出位置观测波形
     * @note    2 : 输出转速观测波形
     * @note    3 : 输出alpha-beta轴高频电压波形
     * @note    4 : 输出alpha-beta轴电流波形(原始值)
     * @note    5 : 输出alpha-beta轴高频电流波形
     * @note    6 : 输出dq轴电流波形
     * @note    7 : 输出PLL输入的误差信号
     */
    if (system_dac_print == 0) {
        u_dac_value1 = (uint16_t)(2048);
        u_dac_value2 = (uint16_t)(2048);
    } else if (system_dac_print == 1) {
        u_dac_value1 = (uint16_t)((Drive_hfi.theta_true + M_PI) / (2 * M_PI) * 4096);
        u_dac_value2 = (uint16_t)((Drive_hfi.theta_obs + M_PI) / (2 * M_PI) * 4096);
    } else if (system_dac_print == 2) {
        u_dac_value1 = (uint16_t)((Drive_hfi.speed_true + 80.0f) / (160.0f) * 4096);
        u_dac_value2 = (uint16_t)((Drive_hfi.speed_obs + 80.0f) / (160.0f) * 4096);
    } else if (system_dac_print == 3) {
        u_dac_value1 = (uint16_t)((Drive_ualphabetah.alpha + 200.0f) / (400.0f) * 4096);
        u_dac_value2 = (uint16_t)((Drive_ualphabetah.beta + 200.0f) / (400.0f) * 4096);
    } else if (system_dac_print == 4) {
        u_dac_value1 = (uint16_t)((Drive_ialphabeta.alpha + 4.5f) / (9.0f) * 4096);
        u_dac_value2 = (uint16_t)((Drive_ialphabeta.beta + 4.5f) / (9.0f) * 4096);
    } else if (system_dac_print == 5) {
        u_dac_value1 = (uint16_t)((Drive_hfi.i_alpha_beta_h.alpha + 0.25f) / (0.5f) * 4096);
        u_dac_value2 = (uint16_t)((Drive_hfi.i_alpha_beta_h.beta + 0.25f) / (0.5f) * 4096);
    } else if (system_dac_print == 6) {
        u_dac_value1 = (uint16_t)((Drive_idq.d + 4.5f) / (9.0f) * 4096);
        u_dac_value2 = (uint16_t)((Drive_idq.q + 4.5f) / (9.0f) * 4096);
    } else if (system_dac_print == 7) {
        u_dac_value1 = (uint16_t)((Drive_hfi.error_dem + 0.25f) / (0.5f) * 4096);
    }
}