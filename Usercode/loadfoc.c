#include "loadfoc.h"

/**
 * @brief   Load Motor FOC Control caculate, use it in interrupt
 */
void load_foc_calc(void)
{
    /************************************************************
     * @brief   FOC Caculate
     */

    // Current loop
    // current ABC-to-dq
    abc_2_dq(&Load_iabc, &Load_idq, Load_AD2S.Electrical_Angle);

    // (id=0 control)Current PI Controller
    // d-axis
    Load_id_pi.ref = 0;
    Load_id_pi.fdb = Load_idq.d;
    PID_Calc(&Load_id_pi, system_enable, SYSTEM_SAMPLE_TIME);
    Load_udq.d = Load_id_pi.output;

    // q-axis
    Load_iq_pi.ref = load_iq_ref;
    Load_iq_pi.fdb = Load_idq.q;
    PID_Calc(&Load_iq_pi, system_enable, SYSTEM_SAMPLE_TIME);
    Load_udq.q = Load_iq_pi.output;

    /************************************************************
     * @brief   SVPWM
     */
    dq_2_abc(&Load_udq, &Load_uabc, Load_AD2S.Electrical_Angle);
    e_svpwm(&Load_uabc, 101, &Load_duty_abc);
}