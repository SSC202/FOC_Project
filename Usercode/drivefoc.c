#include "drivefoc.h"

/**
 * @brief   Drive Motor FOC Control caculate, use it in interrupt
 */
void drive_foc_calc(void)
{
    /************************************************************
     * @brief   HFI Calculate
     */

    // current ABC-to-dq
    abc_2_dq(&Drive_iabc, &Drive_idq_hat, Drive_hfi.theta_obs);

    // current HPF
    Drive_id_hat_filter.input = Drive_idq_hat.d;
    Drive_iq_hat_filter.input = Drive_idq_hat.q;
    LPF_Calc(&Drive_id_hat_filter, system_enable);
    LPF_Calc(&Drive_iq_hat_filter, system_enable);
    Drive_idqh.d = Drive_idq_hat.d - Drive_id_hat_filter.output;
    Drive_idqh.q = Drive_idq_hat.q - Drive_iq_hat_filter.output;

    // HFI Caculate
    Drive_hfi.theta_true = Drive_AD2S.Electrical_Angle;
    Drive_hfi.speed_true = Drive_AD2S.Speed;
    Drive_hfi.idqh_now.d = Drive_idqh.d;
    Drive_hfi.idqh_now.q = Drive_idqh.q;
    HFI_Calc(&Drive_hfi);

    /************************************************************
     * @brief   FOC Calculate
     */

    // Speed loop
    Drive_speed_pi.ref = drive_speed_ref;
    // Drive_speed_pi.fdb = Drive_AD2S.Speed;
    Drive_speed_pi.fdb = Drive_hfi.speed_obs;
    PID_Calc(&Drive_speed_pi, system_enable, SYSTEM_SAMPLE_TIME);

    // Current loop
    // current ABC-to-dq
    // abc_2_dq(&Drive_iabc, &Drive_idq, Drive_AD2S.Electrical_Angle);
    abc_2_dq(&Drive_iabc, &Drive_idq, Drive_hfi.theta_obs);

    // current LPF
    Drive_id_filter.input = Drive_idq.d;
    Drive_iq_filter.input = Drive_idq.q;
    LPF_Calc(&Drive_id_filter, system_enable);
    LPF_Calc(&Drive_iq_filter, system_enable);
    Drive_idql.d = Drive_id_filter.output;
    Drive_idql.q = Drive_iq_filter.output;

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
    Drive_udqh.d = Drive_hfi.udq_h.d;
    Drive_udqh.q = Drive_hfi.udq_h.q;
    dq_2_abc(&Drive_udqh, &Drive_uabch, Drive_hfi.theta_obs);

    /************************************************************
     * @brief   SVPWM
     */
    Drive_uabc.a = Drive_uabcl.a + Drive_uabch.a;
    Drive_uabc.b = Drive_uabcl.b + Drive_uabch.b;
    Drive_uabc.c = Drive_uabcl.c + Drive_uabch.c;
    e_svpwm(&Drive_uabc, 201, &Drive_duty_abc);
}