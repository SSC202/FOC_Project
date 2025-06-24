#include "userinit.h"

/**
 * @brief   Userinit
 */
void user_init(void)
{
    // Init HFI parameters
    LPF_Init(&Drive_id_filter, 200, SYSTEM_SAMPLE_TIME);
    LPF_Init(&Drive_iq_filter, 200, SYSTEM_SAMPLE_TIME);
    LPF_Init(&Drive_id_hat_filter, 200, SYSTEM_SAMPLE_TIME);
    LPF_Init(&Drive_iq_hat_filter, 200, SYSTEM_SAMPLE_TIME);               // Current LPF Init
    HFI_Init(&Drive_hfi, 0.2, SYSTEM_SAMPLE_TIME, 800, 8000, 10, 0.00227); // offset: [3]-[1](0.005)
    LPF_Init(&Drive_ish_filter, 1, SYSTEM_SAMPLE_TIME);
}
