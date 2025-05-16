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
    LPF_Init(&Drive_iq_hat_filter, 200, SYSTEM_SAMPLE_TIME); // Current LPF Init
    HFI_Init(&Drive_hfi, 50, SYSTEM_SAMPLE_TIME * 2, 200, 1000, 10, 0.005);
}
