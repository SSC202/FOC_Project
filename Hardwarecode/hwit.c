#include "hwit.h"

void hw_curr_sample(ADC_HandleTypeDef *hadc)
{
    static int adc_cnt                = 0;
    static uint32_t adc_u1_offset_sum = 0;
    static uint32_t adc_v1_offset_sum = 0;
    static uint32_t adc_u2_offset_sum = 0;
    static uint32_t adc_v2_offset_sum = 0;
    UNUSED(hadc);
    /**********************************
     * @brief   Sample
     */
    if (hadc == &hadc1) {
        // Read Offset
        if (Drive_curr.sample_flag == CURR_SAMPLE_GET_OFFSET) {
            adc_u1_offset_sum += hadc1.Instance->JDR1;
            adc_v1_offset_sum += hadc1.Instance->JDR2;
            adc_u2_offset_sum += hadc1.Instance->JDR3;
            adc_v2_offset_sum += hadc1.Instance->JDR4;
            adc_cnt++;
            if (adc_cnt == 20) {
                adc_cnt                     = 0;
                Drive_curr.adc_val_u_offset = adc_u1_offset_sum / 20.0f;
                Drive_curr.adc_val_v_offset = adc_v1_offset_sum / 20.0f;
                Drive_curr.sample_flag      = CURR_SAMPLE_RUNNING;
                adc_u1_offset_sum           = 0;
                adc_v1_offset_sum           = 0;
                Load_curr.adc_val_u_offset  = adc_u2_offset_sum / 20.0f;
                Load_curr.adc_val_v_offset  = adc_v2_offset_sum / 20.0f;
                Load_curr.sample_flag       = CURR_SAMPLE_RUNNING;
                adc_u2_offset_sum           = 0;
                adc_v2_offset_sum           = 0;
            }
        }
        // Read Current
        else {
            Drive_curr.adc_val_u = hadc1.Instance->JDR1;
            Drive_curr.adc_val_v = hadc1.Instance->JDR2;
            Load_curr.adc_val_u  = hadc1.Instance->JDR3;
            Load_curr.adc_val_v  = hadc1.Instance->JDR4;
            adc_2_curr(&Drive_curr);
            adc_2_curr(&Load_curr);
            Drive_iabc.a = Drive_curr.curr_u;
            Drive_iabc.b = Drive_curr.curr_v;
            Drive_iabc.c = Drive_curr.curr_w;
            Load_iabc.a  = Load_curr.curr_u;
            Load_iabc.b  = Load_curr.curr_v;
            Load_iabc.c  = Load_curr.curr_w;
            // software protection
            if ((Drive_iabc.a * Drive_iabc.a > DRIVE_MAX_CURRENT * DRIVE_MAX_CURRENT) || (Drive_iabc.b * Drive_iabc.b > DRIVE_MAX_CURRENT * DRIVE_MAX_CURRENT) || (Drive_iabc.c * Drive_iabc.c > DRIVE_MAX_CURRENT * DRIVE_MAX_CURRENT) || (Load_iabc.a * Load_iabc.a > LOAD_MAX_CURRENT * LOAD_MAX_CURRENT) || (Load_iabc.b * Load_iabc.b > LOAD_MAX_CURRENT * LOAD_MAX_CURRENT) || (Load_iabc.c * Load_iabc.c > LOAD_MAX_CURRENT * LOAD_MAX_CURRENT)) {
                system_enable = 0;
            }
        }
    }
}