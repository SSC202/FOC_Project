#include "usermain.h"

/**
 * @brief   串口重定向函数
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

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
    PID_init(&Drive_speed_pi, 0.1, 0.1, 0, 1); // Speed PI Init
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
            printf("U:%f\r\n", Drive_AD2S.Electrical_Angle);
        }
        if (system_print == 1) {
            printf("U:%f\r\n", Drive_AD2S.Speed);
        }
    }
}



static void load_foc_calc(void)
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

/**
 * @brief   ADC Injected Channel interrupt Callback function
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static int adc_cnt                = 0;
    static uint32_t adc_u1_offset_sum = 0;
    static uint32_t adc_v1_offset_sum = 0;
    static uint32_t adc_u2_offset_sum = 0;
    static uint32_t adc_v2_offset_sum = 0;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, 0); // Caculate running time
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
            if ((Drive_iabc.a * Drive_iabc.a > MAX_CURRENT * MAX_CURRENT) || (Drive_iabc.b * Drive_iabc.b > MAX_CURRENT * MAX_CURRENT) || (Drive_iabc.c * Drive_iabc.c > MAX_CURRENT * MAX_CURRENT) || (Load_iabc.a * Load_iabc.a > MAX_CURRENT * MAX_CURRENT) || (Load_iabc.b * Load_iabc.b > MAX_CURRENT * MAX_CURRENT) || (Load_iabc.c * Load_iabc.c > MAX_CURRENT * MAX_CURRENT)) {
                system_enable = 0;
            }
        }
    }
    // Angle and Speed Sample
    AD2S1210_Angle_Get();                   // Angle Sample
    AD2S1210_Speed_Get(SYSTEM_SAMPLE_TIME); // Speed Sample
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