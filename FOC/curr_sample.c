
#include "curr_sample.h"

/********************************** 用户函数段 **************************************** */

/**
 * @brief   将 ADC 值转换为电流值
 * @param   curr_sample     电机采样电流结构体指针
 */
void adc_2_curr(curr_sample_t *curr_sample)
{
    curr_sample->curr_u = ((curr_sample->adc_val_u - curr_sample->adc_val_u_offset) / 65535.f) * 9.6f * 3.3f;
    curr_sample->curr_v = ((curr_sample->adc_val_v - curr_sample->adc_val_v_offset) / 65535.f) * 9.6f * 3.3f;
    curr_sample->curr_w = -curr_sample->curr_u - curr_sample->curr_v;
}
