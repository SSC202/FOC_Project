#include "drivefoc.h"

/**
 * @brief   Drive Motor FOC Control caculate, use it in interrupt
 */
void drive_foc_calc(void)
{
    /************************************************************
     * @brief   FOC Caculate
     */

    // Speed loop
    Drive_speed_pi.ref = drive_speed_ref;
    // Drive_speed_pi.fdb = Drive_AD2S.Speed;
    Drive_speed_pi.fdb = Drive_AD2S.Speed;
    PID_Calc(&Drive_speed_pi, system_enable, SYSTEM_SAMPLE_TIME);

    // Current loop
    // current ABC-to-dq
    abc_2_dq(&Drive_iabc, &Drive_idq, Drive_AD2S.Electrical_Angle);

    // (id=0 control)Current PI Controller
    // d-axis
    Drive_id_pi.ref = 0;
    Drive_id_pi.fdb = Drive_idq.d;
    PID_Calc(&Drive_id_pi, system_enable, SYSTEM_SAMPLE_TIME);
    Drive_udq.d = Drive_id_pi.output;

    // q-axis
    Drive_iq_pi.ref = Drive_speed_pi.output;
    Drive_iq_pi.fdb = Drive_idq.q;
    PID_Calc(&Drive_iq_pi, system_enable, SYSTEM_SAMPLE_TIME);
    Drive_udq.q = Drive_iq_pi.output;

    /************************************************************
     * @brief   SVPWM
     */
    dq_2_abc(&Drive_udq, &Drive_uabc, Drive_AD2S.Electrical_Angle);
    e_svpwm(&Drive_uabc, 101, &Drive_duty_abc);
}