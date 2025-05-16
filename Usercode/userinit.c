#include "userinit.h"

/**
 * @brief   Userinit
 */
void user_init(void)
{
    HFI_Init(&Drive_hfi, 50, SYSTEM_SAMPLE_TIME * 2, 200, 1000, 50, 0.00225);
}
