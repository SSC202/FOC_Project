#include "hwinit.h"

/**
 * @brief   Hardware Init
 */
void hw_init(void)
{
    // Wait
    HAL_Delay(50);
    // Init AD2S1210
    AD2S1210_Init();
    // system init
    system_enable = 0;
    // Init FOC parameters
    // Drive Motor
    drive_speed_ref             = 0;
    Drive_curr.adc_val_u_offset = 0;
    Drive_curr.adc_val_v_offset = 0;
    Drive_curr.sample_flag      = CURR_SAMPLE_GET_OFFSET; // Read ADC Offset
    PID_init(&Drive_id_pi, 0.005, 12.5, 0, 80);           // Current PI Init
    PID_init(&Drive_iq_pi, 0.005, 12.5, 0, 80);
    PID_init(&Drive_speed_pi, 0.1, 100, 0, 2); // Speed PI Init
    // Load Motor
    load_iq_ref                = 0;
    Load_curr.sample_flag      = CURR_SAMPLE_GET_OFFSET; // Read ADC Offset
    Load_curr.adc_val_u_offset = 0;
    Load_curr.adc_val_v_offset = 0;
    PID_init(&Load_id_pi, 0.005, 12.5, 0, 80); // Current PI Init
    PID_init(&Load_iq_pi, 0.005, 12.5, 0, 80);
    // Current Sample
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // Wait for Read ADC Offset
    while (Drive_curr.sample_flag == CURR_SAMPLE_GET_OFFSET || Load_curr.sample_flag == CURR_SAMPLE_GET_OFFSET) {
        ;
    }
    // DAC Init
    HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
    AD5676_init();
    // Open Inverter
    HAL_TIM_Base_Start_IT(&htim8);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
