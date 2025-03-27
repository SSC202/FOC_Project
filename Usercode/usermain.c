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
uint8_t system_print     = 0;       // 参数打印
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

float load_speed_ref; // 速度指令值(斜坡)

LPF_t Load_ide_filter; // 估计 d 轴电流低通滤波器
LPF_t Load_iqe_filter; // 估计 q 轴电流低通滤波器
LPF_t Load_idt_filter; // 实际 d 轴电流低通滤波器
LPF_t Load_iqt_filter; // 实际 q 轴电流低通滤波器

hfi_t Load_hfi;   // 负载电机高频注入结构体
dq_t Load_udqh;   // 负载电机高频注入 dq 轴电压
abc_t Load_uabch; // 负载电机高频注入 ABC 相电压

dq_t Load_idqe; // 估计dq电流
dq_t Load_idqt; // 实际dq电流

dq_t Load_idqh;   // 负载电机高频注入 dq 轴电流
abc_t Load_iabch; // 负载电机高频注入 ABC 相电流

abc_t Load_iabcl; // 负载电机 ABC 相基波电流
dq_t Load_idql;   // 负载电机 dq 轴基波电流

float hfi_power;                // 高频注入输入功率
float sum_hfi_power;            // 功率和
float average_hfi_power;        // 功率均值
// 以下为 Debug 临时变量
uint16_t u_dac_value;

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
    Load_udq.d                 = 0;
    Load_udq.q                 = 0;
    load_speed_ref             = 0;
    Load_curr.adc_val_u_offset = 0;
    Load_curr.adc_val_v_offset = 0;
    Load_curr.sample_flag      = CURR_SAMPLE_GET_OFFSET; // Read ADC Offset
    PID_init(&Load_id_pi, 0.005, 12.5, 0, 80);           // Current PI Init
    PID_init(&Load_iq_pi, 0.005, 12.5, 0, 80);
    PID_init(&Load_speed_pi, 0.1, 0.1, 0, 1); // Speed PI Init
    LPF_Init(&Load_ide_filter, 200, system_sample_time);
    LPF_Init(&Load_iqe_filter, 200, system_sample_time);
    LPF_Init(&Load_idt_filter, 200, system_sample_time);
    LPF_Init(&Load_iqt_filter, 200, system_sample_time); // Current LPF Init
    // Init HFI parameters
    // AD2S1210_Angle_Get();
    // observer_Init(&Load_hfi, 50, 0, system_sample_time, 1, 1, 200, 1000);
	observer_Init(&Load_hfi, 50, 1.57, system_sample_time, 1, 1, 200, 1000);
    // Power caculate init
    sum_hfi_power = 0;
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
        if (system_print == 0)
            printf("U:%f,%f\n", Load_AD2S.Electrical_Angle, Load_hfi.electric_theta_obs);
        else if (system_print == 1)
            printf("U:%f\n", average_hfi_power);
    }
}

/**
 * @brief   FOC计算程序
 */
static void foc_calc(void)
{
    // 角度和速度采样
    AD2S1210_Angle_Get();
    AD2S1210_Speed_Get(system_sample_time);

    /**
     * HFI 计算
     */
    abc_2_dq(&Load_iabc, &Load_idqe, Load_hfi.electric_theta_obs);
    // dq 轴 LPF
    Load_ide_filter.input = Load_idqe.d;
    Load_iqe_filter.input = Load_idqe.q;
    LPF_Calc(&Load_ide_filter);
    LPF_Calc(&Load_iqe_filter);
    Load_idqh.d = Load_idqe.d - Load_ide_filter.output;
    Load_idqh.q = Load_idqe.q - Load_iqe_filter.output;

    Load_hfi.idh_last = Load_hfi.idh;
    Load_hfi.iqh_last = Load_hfi.iqh;
    Load_hfi.idh      = Load_idqh.d;
    Load_hfi.iqh      = Load_idqh.q;
    Load_hfi.delta_id = Load_hfi.idh - Load_hfi.idh_last;
    Load_hfi.delta_iq = Load_hfi.iqh - Load_hfi.iqh_last;
    // 每一拍进行功率计算
    hfi_power = fabs(Load_udqh.d * Load_idqh.d) + fabs(Load_udqh.q * Load_idqh.q);
    sum_hfi_power = sum_hfi_power + hfi_power;
    if (Load_hfi.clap == 4) {
        average_hfi_power = sum_hfi_power / 4.f;
        sum_hfi_power = 0;
        u_dac_value = ((average_hfi_power)/(5)) * 2048 + 2048;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, u_dac_value);
    }
    // 在第二拍和第四拍时进行角度计算
    if (Load_hfi.clap == 2 || Load_hfi.clap == 4) {
        observer_Calc(&Load_hfi, system_enable);
    }
    if(system_enable == 0)
    {
        Load_hfi.electric_theta_obs = Load_AD2S.Electrical_Angle;
    }
    

    /**
     * FOC 计算
     */
    abc_2_dq(&Load_iabc, &Load_idqt, Load_AD2S.Electrical_Angle);
    Load_idt_filter.input = Load_idqt.d;
    Load_iqt_filter.input = Load_idqt.q;
    LPF_Calc(&Load_idt_filter);
    LPF_Calc(&Load_iqt_filter);
    Load_idql.d = Load_idt_filter.output;
    Load_idql.q = Load_iqt_filter.output;

    // 速度 PI 控制器计算
    Load_speed_pi.ref = load_speed_ref;
    Load_speed_pi.fdb = Load_AD2S.Speed;
    PID_Calc(&Load_speed_pi, system_enable, system_sample_time);

    // 电流 PI 控制器计算(id = 0)
    // d轴
    Load_id_pi.ref = 0;
    Load_id_pi.fdb = Load_idql.d;
    PID_Calc(&Load_id_pi, system_enable, system_sample_time);
    Load_udq.d = Load_id_pi.output;

    // q轴
    Load_iq_pi.ref = Load_speed_pi.output;
    Load_iq_pi.fdb = Load_idql.q;
    PID_Calc(&Load_iq_pi, system_enable, system_sample_time);
    Load_udq.q = Load_iq_pi.output;

    dq_2_abc(&Load_udq, &Load_uabc, Load_AD2S.Electrical_Angle);

    /**
     * HFI 注入
     */
    // HFI 状态
    switch (Load_hfi.clap) {
        // 第一拍
        case 1:
            Load_hfi.inject_counter = 1;
            Load_hfi.clap           = 2;
            break;
        // 第二拍
        case 2:
            Load_hfi.inject_counter = -1;
            Load_hfi.clap           = 3;
            break;
        // 第三拍
        case 3:
            Load_hfi.inject_counter = -1;
            Load_hfi.clap           = 4;
            break;
        // 第四拍
        case 4:
            Load_hfi.inject_counter = 1;
            Load_hfi.clap           = 1;
            break;
        default:
            Load_hfi.inject_counter = 1;
            Load_hfi.clap           = 1;
            break;
    }
    Load_udqh.d = Load_hfi.u_h * cosf(Load_hfi.electric_theta_inj) * Load_hfi.inject_counter;
    Load_udqh.q = Load_hfi.u_h * sinf(Load_hfi.electric_theta_inj) * Load_hfi.inject_counter;
    dq_2_abc(&Load_udqh, &Load_uabch, Load_hfi.electric_theta_obs);

    // SVPWM
    Load_uabc.a = Load_uabc.a + Load_uabch.a;
    Load_uabc.b = Load_uabc.b + Load_uabch.b;
    Load_uabc.c = Load_uabc.c + Load_uabch.c;
    e_svpwm(&Load_uabc, 201, &Load_duty_abc);
}

/**
 * @brief   定时器中断
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM8) {
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
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, 0);
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
    foc_calc();
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
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, 1);
}