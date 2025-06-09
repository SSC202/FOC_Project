#include "usermain.h"

/**
 * @brief   Init Program
 */
static void init(void)
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
    // Userinit
    user_init();
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

/**
 * @brief   主函数
 */
void usermain(void)
{
    init();
    while (1) {
        // DAC Print
        // Usart Print
        if (system_print == 0) {
            printf("U:%f,%f\r\n", Drive_AD2S.Electrical_Angle, Drive_hfi.theta_obs);
        } else if (system_print == 1) {
            printf("U:%f,%f\r\n", Drive_AD2S.Speed, Drive_hfi.speed_obs);
        } else if (system_print == 2) {
            printf("U:%f,%f\r\n", Drive_ish_filter.output, Drive_hfi.ish);
        } else if (system_print == 3) {
            printf("U:%f\r\n", Drive_hfi.u_h);
        }
    }
}

/**
 * @brief   ADC Injected Channel interrupt Callback function
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0); // Caculate running time

    /**********************************
     * @brief   Sample Calculate
     */
    hw_curr_sample(hadc);

    // Angle and Speed Sample
    AD2S1210_Angle_Get(); // Angle Sample
    AD2S1210_Speed_Get(); // Speed Sample
    /**********************************
     * @brief   FOC Calculate
     */
    drive_foc_calc();
    load_foc_calc();

    /**********************************
     * @brief   Voltage-Source Inverter Control
     */
    if (system_enable == 0) {
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
    } else {
        TIM8->CCR1 = Drive_duty_abc.dutya * TIM8->ARR;
        TIM8->CCR2 = Drive_duty_abc.dutyb * TIM8->ARR;
        TIM8->CCR3 = Drive_duty_abc.dutyc * TIM8->ARR;
        TIM1->CCR1 = Load_duty_abc.dutya * TIM1->ARR;
        TIM1->CCR2 = Load_duty_abc.dutyb * TIM1->ARR;
        TIM1->CCR3 = Load_duty_abc.dutyc * TIM1->ARR;
    }

    /**********************************
     * @brief   DAC Output
     */
    hw_dac_output();
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, system_dac_value1);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, system_dac_value2);

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 1); // Caculate running time
}

/**
 * @brief   Timer interrupt
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) {
    }
}