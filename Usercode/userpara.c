#include "userpara.h"

/******************************************************
 * @brief   运行相关的参数,结合 CubeMX 进行定义
 */

#define DRIVE_MAX_CURRENT  8.0f    // 驱动电机电流限制
#define LOAD_MAX_CURRENT   20.0f   // 驱动电机电流限制
#define SYSTEM_SAMPLE_TIME 0.0001f // 系统采样时间,根据 CubeMX 配置

uint8_t system_enable;    // 系统使能参数,0为失能,1为使能
uint8_t system_print;     // 参数打印,用户自行定义串口打印的变量
uint8_t system_dac_print; // DAC打印，用户自行定义DAC打印的变量

uint16_t system_dac_value1;   // DAC Channel1 示波器打印变量,用户自行赋值
uint16_t system_dac_value2;   // DAC Channel2 示波器打印变量,用户自行赋值
extern uint16_t hwdac_value1; // 外部DAC1通道值
extern uint16_t hwdac_value2; // 外部DAC2通道值
extern uint16_t hwdac_value3; // 外部DAC3通道值
extern uint16_t hwdac_value4; // 外部DAC4通道值

/******************************************************
 * @brief   临时变量
 */

/******************************************************
 * @brief   对拖平台驱动电机端相关定义
 * @note    驱动电机变量命名使用 Drive_xxx 格式, 结构体使用 Drive, 单变量使用 drive
 */
// 速度环 PID 相关定义
PID_t Drive_speed_pi; // 驱动电机速度PI控制器结构体

// 电流环 PID 相关定义
PID_t Drive_id_pi; // 驱动电机d轴电流PI控制器结构体
PID_t Drive_iq_pi; // 驱动电机q轴电流PI控制器结构体

// SVPWM 输出相关定义
dq_t Drive_udql;   // 驱动电机 dq 轴低频指令电压
abc_t Drive_uabcl; // 驱动电机 ABC 相低频指令电压
abc_t Drive_uabc;  // 驱动电机 ABC 相指令电压

duty_abc_t Drive_duty_abc; // 驱动电机 ABC 相占空比值

// 电流采样相关定义
curr_sample_t Drive_curr; // 驱动电机采样电流相关结构体

abc_t Drive_iabc; // 驱动电机 ABC 相电流(原始值)
dq_t Drive_idq;   // 驱动电机 dq 轴电流(原始值)

LPF_t Drive_id_filter; // 驱动电机 d 轴电流低通滤波器
LPF_t Drive_iq_filter; // 驱动电机 q 轴电流低通滤波器
dq_t Drive_idql;       // 驱动电机估计 dq 轴低频电流

// 速度/位置采样相关定义
extern ad2s1210_t Drive_AD2S; // 驱动电机的旋变解码板结构体

float drive_speed_ref; // 速度指令值

/******************************************************
 * @brief   对拖平台负载电机端相关定义
 * @note    负载电机变量命名使用 Load_xxx 格式, 结构体使用 Load, 单变量使用 load
 */

// 电流环 PID 相关定义
PID_t Load_id_pi; // 负载电机d轴电流PI控制器结构体
PID_t Load_iq_pi; // 负载电机q轴电流PI控制器结构体

// SVPWM 输出相关定义
dq_t Load_udq;            // 负载电机 dq 轴指令电压
abc_t Load_uabc;          // 负载电机 ABC 相指令电压
duty_abc_t Load_duty_abc; // 负载电机 ABC 相占空比值

// 电流采样相关定义
curr_sample_t Load_curr; // 负载电机采样电流相关结构体

abc_t Load_iabc; // 负载电机 ABC 相电流(原始值)
dq_t Load_idq;   // 负载电机 dq 轴电流(原始值)

// 位置采样相关定义
extern ad2s1210_t Load_AD2S; // 负载电机的旋变解码板结构体

float load_iq_ref; // 电流指令值

/****************************************
 * @brief   以下为无感算法变量相关定义,用户自行定义相关全局变量
 */
dq_t Drive_idq_hat;        // 驱动电机估计 dq 轴电流
LPF_t Drive_id_hat_filter; // 驱动电机估计 d 轴电流低通滤波器
LPF_t Drive_iq_hat_filter; // 驱动电机估计 q 轴电流低通滤波器
dq_t Drive_idqh;           // 驱动电机估计 dq 轴高频电流

HFI_t Drive_hfi;   // 驱动电机高频注入结构体
dq_t Drive_udqh;   // 驱动电机估计 dq 轴高频指令电压
abc_t Drive_uabch; // 驱动电机 ABC 相高频指令电压

LPF_t Drive_ish_filter;
