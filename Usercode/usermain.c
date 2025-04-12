#include "usermain.h"

/**
 * @brief   串口重定向函数
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

/******************************************************
 * @brief   运行相关的参数,结合 CubeMX 进行定义
 */
uint8_t system_enable    = 0;       // 系统使能参数,0为失能,1为使能
uint8_t system_print     = 0;       // 参数打印,用户自行定义串口打印的变量
float system_sample_time = 0.0001f; // 系统采样时间,根据 CubeMX 配置
uint16_t u_dac_value;               // DAC 示波器打印变量,用户自行赋值

/******************************************************
 * @brief   临时变量
 */
// float error = 0;
// u_dac_value = (uint16_t)((Drive_hfi.i_err * 20000 * 200 / 2048) + 2048);
// HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, u_dac_value);

/******************************************************
 * @brief   对拖平台驱动电机端相关定义
 * @note    驱动电机变量命名使用 Drive_xxx 格式, 结构体使用 Drive, 单变量使用 drive
 */
// 速度环 PID 相关定义
PID_t Drive_speed_pi; // 驱动电机速度PI控制器结构体

// 电流环 PID 相关定义
PID_t Drive_id_pi; // 驱动电机d轴电流PI控制器结构体
PID_t Drive_iq_pi; // 驱动电机q轴电流PI控制器结构体

// FOC dq-ABC 相关定义
dq_t Drive_udql; // 驱动电机 dq 轴低频指令电压

// SVPWM 输出相关定义
dq_t Drive_udq;            // 驱动电机 dq 轴指令电压
abc_t Drive_uabc;          // 驱动电机 ABC 相指令电压
duty_abc_t Drive_duty_abc; // 驱动电机 ABC 相占空比值

// 电流采样相关定义
curr_sample_t Drive_curr; // 驱动电机采样电流相关结构体

abc_t Drive_iabc; // 驱动电机 ABC 相电流(原始值)
dq_t Drive_idq;   // 驱动电机 dq 轴电流(原始值)

// 电流低通滤波器
LPF_t Drive_id_filter; // d 轴电流低通滤波器
LPF_t Drive_iq_filter; // q 轴电流低通滤波器

dq_t Drive_idql; // 驱动电机 dq 轴基波电流

// 速度/位置采样相关定义
extern ad2s1210_t Drive_AD2S; // 驱动电机的旋变解码板结构体

float drive_speed_ref; // 速度指令值

/******************************************************
 * @brief   对拖平台负载电机端相关定义
 * @note    驱动电机变量命名使用 Load_xxx 格式, 结构体使用 Load, 单变量使用 load
 */

/****************************************
 * @brief   以下为无感算法变量相关定义,用户自行定义相关全局变量
 */


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
    drive_speed_ref             = 0;
    Drive_curr.adc_val_u_offset = 0;
    Drive_curr.adc_val_v_offset = 0;
    Drive_curr.sample_flag      = CURR_SAMPLE_GET_OFFSET; // Read ADC Offset
    PID_init(&Drive_id_pi, 0.005, 12.5, 0, 80);           // Current PI Init
    PID_init(&Drive_iq_pi, 0.005, 12.5, 0, 80);
    PID_init(&Drive_speed_pi, 0.1, 0.1, 0, 1); // Speed PI Init
		LPF_Init(&Drive_id_filter, 200, system_sample_time); // Idq LPF init
    LPF_Init(&Drive_iq_filter, 200, system_sample_time);
    // Current Sample
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // Wait for Read ADC Offset
    while (Drive_curr.sample_flag == CURR_SAMPLE_GET_OFFSET) {
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
    }
}

/**
 * @brief   FOC Control caculate, use it in interrupt
 */
static void foc_calc(void)
{
    // Angle and Speed Sample
    AD2S1210_Angle_Get();                   // Angle Sample
    AD2S1210_Speed_Get(system_sample_time); // Speed Sample
    /************************************************************
     * @brief   HFI Caculate
     */
    
    /************************************************************
     * @brief   FOC Caculate
     */

    // Speed loop
    Drive_speed_pi.ref = drive_speed_ref;
    // Drive_speed_pi.fdb = Drive_AD2S.Speed;
    Drive_speed_pi.fdb = Drive_AD2S.Speed;
    PID_Calc(&Drive_speed_pi, system_enable, system_sample_time);

    // Current loop
    // current ABC-to-dq
    abc_2_dq(&Drive_iabc, &Drive_idq, Drive_AD2S.Electrical_Angle);

    // current LPF
    Drive_id_filter.input = Drive_idq.d;
    Drive_iq_filter.input = Drive_idq.q;
    LPF_Calc(&Drive_id_filter);
    LPF_Calc(&Drive_iq_filter);
    Drive_idql.d = Drive_id_filter.output;
    Drive_idql.q = Drive_iq_filter.output;

    // (id=0 control)Current PI Controller
    // d-axis
    Drive_id_pi.ref = 0;
    Drive_id_pi.fdb = Drive_idq.d;
    PID_Calc(&Drive_id_pi, system_enable, system_sample_time);
    Drive_udql.d = Drive_id_pi.output;

    // q-axis
    Drive_iq_pi.ref = Drive_speed_pi.output;
    Drive_iq_pi.fdb = Drive_idq.q;
    PID_Calc(&Drive_iq_pi, system_enable, system_sample_time);
    Drive_udql.q = Drive_iq_pi.output;

    // voltage dq-to-ABC
    // dq_2_abc(&Drive_udql, &Drive_uabcl, Drive_AD2S.Electrical_Angle);

    /************************************************************
     * @brief   SVPWM
     */
    Drive_udq.d = Drive_udql.d;
    Drive_udq.q = Drive_udql.q;
    dq_2_abc(&Drive_udq, &Drive_uabc, Drive_AD2S.Electrical_Angle);
    e_svpwm(&Drive_uabc, 201, &Drive_duty_abc);
}

/**
 * @brief   ADC Injected Channel interrupt Callback function
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static int adc_cnt               = 0;
    static uint32_t adc_u_offset_sum = 0;
    static uint32_t adc_v_offset_sum = 0;
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, 0); // Caculate running time
    UNUSED(hadc);
    /**********************************
     * @brief   Current sample
     */
    if (hadc == &hadc1) {
        // Read Offset
        if (Drive_curr.sample_flag == CURR_SAMPLE_GET_OFFSET) {
            adc_u_offset_sum += hadc1.Instance->JDR1;
            adc_v_offset_sum += hadc1.Instance->JDR2;
            adc_cnt++;
            if (adc_cnt == 20) {
                adc_cnt                     = 0;
                Drive_curr.adc_val_u_offset = adc_u_offset_sum / 20.0f;
                Drive_curr.adc_val_v_offset = adc_v_offset_sum / 20.0f;
                Drive_curr.sample_flag      = CURR_SAMPLE_RUNNING;
                adc_u_offset_sum            = 0;
                adc_v_offset_sum            = 0;
            }
        }
        // Read Current
        else {
            Drive_curr.adc_val_u = hadc1.Instance->JDR1;
            Drive_curr.adc_val_v = hadc1.Instance->JDR2;
            adc_2_curr(&Drive_curr);
            Drive_iabc.a = Drive_curr.curr_u;
            Drive_iabc.b = Drive_curr.curr_v;
            Drive_iabc.c = Drive_curr.curr_w;
            // software protection
            if ((Drive_iabc.a * Drive_iabc.a > 1.3 * 1.3) || (Drive_iabc.b * Drive_iabc.b > 1.3 * 1.3) || (Drive_iabc.c * Drive_iabc.c > 1.3 * 1.3)) {
                system_enable = 0;
            }
        }
    }
    /**********************************
     * @brief   FOC Calculate
     */
    foc_calc();
    /**********************************
     * @brief   Voltage-Source Inverter Control
     */
    if (system_enable == 0) {
        TIM8->CCR1 = 0;
        TIM8->CCR2 = 0;
        TIM8->CCR3 = 0;
    } else {
        TIM8->CCR1 = Drive_duty_abc.dutya * TIM8->ARR;
        TIM8->CCR2 = Drive_duty_abc.dutyb * TIM8->ARR;
        TIM8->CCR3 = Drive_duty_abc.dutyc * TIM8->ARR;
    }

    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, 1); // Caculate running time
}

/**
 * @brief   Timer interrupt
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) {
    }
}