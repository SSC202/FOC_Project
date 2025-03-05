#include "usermain.h"

/**
 * @brief   串口重定向函数
 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
    return ch;
}

uint8_t system_enable    = 0;       // 系统使能参数
float system_sample_time = 0.0001f; // 系统采样时间

/**
 *  负载电机相关定义
 */
curr_sample_t Load_curr; // 负载电机采样电流相关结构体
PID_t Load_id_pi;        // 负载电机d轴电流PI控制器结构体
PID_t Load_iq_pi;        // 负载电机q轴电流PI控制器结构体
PID_t Load_speed_pi;     // 负载电机速度PI控制器结构体

duty_abc_t Load_duty_abc; // 负载电机 ABC 相占空比值
dq_t Load_idq;            // 负载电机 dq 轴电流
abc_t Load_iabc;          // 负载电机 ABC 相电流
dq_t Load_udq;            // 负载电机 dq 轴电压
abc_t Load_uabc;          // 负载电机 ABC 相电压

extern ad2s1210_t Load_AD2S; // 负载电机的旋变解码板结构体

float load_speed_ref; // 负载电机速度指令值

/**
 * @brief   Init Program
 */
static void init(void)
{
    // Wait
    HAL_Delay(50);
    // Init AD2S1210
    AD2S1210_Init();
    // Init parameters
    system_enable              = 0;
    Load_udq.d                 = 0;
    Load_udq.q                 = 0;
    load_speed_ref             = 0;
    Load_curr.adc_val_u_offset = 0;
    Load_curr.adc_val_v_offset = 0;
    Load_curr.sample_flag      = CURR_SAMPLE_GET_OFFSET; // Read ADC Offset
    PID_init(&Load_id_pi, 0.02, 50, 0, 50);
    PID_init(&Load_iq_pi, 0.02, 50, 0, 50);
    PID_init(&Load_speed_pi, 0.1, 0.1, 0, 1);
    // Current Sample
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    // Wait for Read ADC Offset
    while (Load_curr.sample_flag == CURR_SAMPLE_GET_OFFSET) {
        ;
    }
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

        // TODO: 主程序里准备一个停止运行的代码段

        // TODO: 同时准备一个恢复运行的代码段，准备将初始化代码移植到此处
        // printf("U:%d,%d,%d\n", cnt[0], cnt[1], cnt[2]);
        // printf("U:%f,%f\n", Load_AD2S.Mechanical_Angle, Load_AD2S.Electrical_Angle);
		printf("U:%f,%f\n", Load_speed_pi.ref, Load_speed_pi.fdb);
    }	
}

/**
 * @brief   定时器中断
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) {

        // Inverter
        if (system_enable == 0) {
            TIM8->CCR1 = 0;
            TIM8->CCR2 = 0;
            TIM8->CCR3 = 0;
        } else {
            TIM8->CCR1 = Load_duty_abc.dutya * TIM8->ARR;
            TIM8->CCR2 = Load_duty_abc.dutyb * TIM8->ARR;
            TIM8->CCR3 = Load_duty_abc.dutyc * TIM8->ARR;
        }
    }
}

/**
 * @brief   ADC 注入组转换中断函数
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    static int adc_cnt               = 0;
    static uint32_t adc_u_offset_sum = 0;
    static uint32_t adc_v_offset_sum = 0;
    UNUSED(hadc);
    if (hadc == &hadc1) {
        // Read Offset
        if (Load_curr.sample_flag == CURR_SAMPLE_GET_OFFSET) {
            adc_u_offset_sum += hadc1.Instance->JDR1;
            adc_v_offset_sum += hadc1.Instance->JDR2;
            adc_cnt++;
            if (adc_cnt == 20) {
                adc_cnt                    = 0;
                Load_curr.adc_val_u_offset = adc_u_offset_sum / 20.0f;
                Load_curr.adc_val_v_offset = adc_v_offset_sum / 20.0f;
                Load_curr.sample_flag      = CURR_SAMPLE_RUNNING;
                adc_u_offset_sum           = 0;
                adc_v_offset_sum           = 0;
            }
        }
        // Read Current
        else {
            Load_curr.adc_val_u = hadc1.Instance->JDR1;
            Load_curr.adc_val_v = hadc1.Instance->JDR2;
            adc_2_curr(&Load_curr);
            Load_iabc.a = Load_curr.curr_u;
            Load_iabc.b = Load_curr.curr_v;
            Load_iabc.c = Load_curr.curr_w;
            // software protection
            if ((Load_iabc.a * Load_iabc.a > 1.3 * 1.3) || (Load_iabc.b * Load_iabc.b > 1.3 * 1.3) || (Load_iabc.c * Load_iabc.c > 1.3 * 1.3)) {
                system_enable = 0;
            }
        }
    }
    // 角度和速度采样
    AD2S1210_Angle_Get();
    AD2S1210_Speed_Get(system_sample_time);

    // 采样电流变换为 dq 轴电流
    abc_2_dq(&Load_iabc, &Load_idq, Load_AD2S.Electrical_Angle);

    // 速度 PI 控制器计算
    Load_speed_pi.ref = load_speed_ref;
    Load_speed_pi.fdb = Load_AD2S.Speed;
    PID_Calc(&Load_speed_pi, system_enable, system_sample_time);

    // 电流 PI 控制器计算(id = 0)
    // d轴
    Load_id_pi.ref = 0;
    Load_id_pi.fdb = Load_idq.d;
    PID_Calc(&Load_id_pi, system_enable, system_sample_time);
    Load_udq.d = Load_id_pi.output;

    // q轴
    Load_iq_pi.ref = Load_speed_pi.output;
    Load_iq_pi.fdb = Load_idq.q;
    PID_Calc(&Load_iq_pi, system_enable, system_sample_time);
    Load_udq.q = Load_iq_pi.output;

    // SVPWM
    dq_2_abc(&Load_udq, &Load_uabc, Load_AD2S.Electrical_Angle);
    e_svpwm(&Load_uabc, 101, &Load_duty_abc);
}