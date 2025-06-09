/*
 * AD2S1210.H
 *
 *  Created on: Dec 4, 2024
 *      Author: WML
 */
/*	注意事项
 *  1.SAMPLE下降沿更新寄存器的值
 *	2.位置和速度数据读取左对齐（即10位分辨率时D15-D6是有效数据）（此处设置使能迟滞只是让低位固定设置为0，禁迟滞时则不管，建议使能，使能后数据更准）
 *	3.故障检测相关：主要通过信号幅值变化，求解是否正常判断故障，用于警告系统当前数据不可信。
 *		①.LOS表示丢失sin或cos信号，信号幅值低于LOS设置的阈值时触发
 *		②.DOS表示信号幅值不正常，低于或高于DOS设置的阈值。
 *		③.LOT表示此时转速超过了量程，解调芯片丢步
 *	输出效果：DOS引脚和LOT引脚有低电平则发生故障
 *		  DOS\LOT
 *		LOS:0\0
 *		DOS:0\1
 *		LOT:1\0
 *	4.	 A0/A1 	配置模式
 *	位置输出:0/0
 *	速度输出:0/1
 *	配置模式:1/1
 *	5.	RES0/RES1	配置分辨率	分辨率和激励信号的频率有推荐关系：
 *	10位：  0/0	10k~20kHz
 *	12位：  0/1	6k~20kHz
 *	14位：  1/0	3k~12kHz
 *	16位：  1/1	2k~10kHz
 *	6.LOS、DOS、LOT等寄存器上电默认值一般不改。我们关注的更多是旋变输出的电压大小是否合适。
 *	7.激励频率设置（AD2S1210始终频率为8.192kHz）(上电时默认10kHz)
 *	激励频率寄存器值=（激励频率*2^15）/AD2S1210时钟频率
 *	8.控制寄存器(默认0x)：
 *		D7:读出控制寄存器的值时，最高位如果是1，则说明读出的数据和上次输入的不一样，即有问题
 *		D6:保留置位1
 *		D5:锁相范围（0=360°，1=44°）
 *		D4:迟滞设置（0=禁用迟滞，1=使能迟滞）
 *		D3:设置编码器输出分辨率EnRES1(编码器输出分辨率设置为<=数字输出分辨率)(默认16位)
 *		D2:EnRES0(配置表和RES0/RES1一样)
 *		D1:设置分辨率RES1（此处数字输出分辨率在配置模式生效）（默认12位）
 *		D0:设置分辨率RES0（普通模式下，数字输出分辨率根据RES0和RES1引脚确定，用户如果进行了模式变化，要确保分辨率一致）
 *	9.软件复位寄存器：软件复位不会覆盖寄存器中的数据，但可用于清除故障寄存器的值，一般用于多个旋变一起复位来同步激励信号相位
 *	10.故障寄存器：高位有效
 *		D7:正弦/余弦输入削波
 *		D6:正弦/余弦输入低于LOS阈值
 *		D5:正弦/余弦输入超过DOS超量程阈值
 *		D4:正弦/余弦输入超过DOS失配阈值
 *		D3:跟踪误差超过LOT阈值
 *		D2:速度超过最大跟踪速率
 *		D1:相位误差超过锁相范围
 *		D0:配置奇偶校验错误
 *	11.SOE=1:并口输出.SOE=0:串行SPI输出
 *	12.SAMPLE:该引脚下降沿触发更新寄存器数据事件
 *	13.数据格式：无符号数表示绝对位置，补码（负数补码为反码+1）表示速度，即带符号读数即可，或理解为MSB表示方向，剩下的是大小。
 *	14.WR/FSYNC（数据锁存脚）：每次数据引脚发送完数据，稳定后，该引脚提供上升沿即可锁定数据。WR/FSYNC的下降沿使SDI和SDO线路脱离高阻态。WR/FSYNC的上升沿使SDI和SDO线路返回高阻态
 *	15.芯片使用时，CS设为低电平
 *	16.写入AD2S1210时，RD保持高电平（并行模式关注）
 *	17.普通模式下用串行接口SPI读取AD2S1210:
 *		①CS始终保持低电平即可（读取前的准备）
 *		②SAMPLE更新数据（读取前的准备）
 *		③根据A0,A1来选择输出的数据（读取前的准备）
 *		④数据默认前16位是数据，后8位是故障，如果不需要故障，在读了16位后的SLCK上升沿之后拉高WR/FSYNC
 *	18.SDO:SCLK上升沿AD2S1210输出数据，故MCU可以在SCLK下降沿读取数据
 *	19.SDI:SCLK下降沿AD2S1210读取数据，故MCU可以在SCLK上升沿时输出数据
 *	20.SDI输入数据时，每次数据输出完后，WR/FSYNC要给一个上升沿锁存数据。
 *	21.故障信息除了软件reset寄存器清除，一般来说用SAMPLE下降沿清除，前提是故障消失。
 *	22.上电时序：硬件RESET引脚置低，延迟60ms以上保证电路稳定后才能进行读取数据等操作
 *	23.位置输出范围（0~360），速度输出范围（+-2500rps）或（+-）5000pi rad/s = 15707.96327 rad/s
 *
 *	并口读数据相关：
 *	24.硬件引脚SOE=1:并口输出
 *	25.并行时，CS全程低电平即可
 *	26.对AD2S1210来说，配置模式下，判断传入的数据是地址（1 ）还是数据（0）根据8bit的MSB（即D7，即时序上第一个传输的数据）决定
 *	27.写入AD2S1210时，RD保持高电平
 *	28.读取AD2S1210时，CS,RD都保持低电平
 *	29.RD低电平时，WR设为高电平
 * */
/**
 * @attention   SPI 配置时:  SPI CPOL = LOW	    //low level
 *                           SPI CPHA = 1	    //second edge
 */
#ifndef __AD2S1210_H
#define __AD2S1210_H

#ifdef __cplusplus
extern "C" {
#endif
/****************************** 硬件/软件SPI定义 ************************************ */

#define Hardware_SPI 1 // 硬件SPI
#define Software_SPI 0 // 软件SPI

#include "stm32h7xx.h"
#include "foc_math.h"
#if (Hardware_SPI == 1)
#include "spi.h"
#endif

/********************************** 寄存器定义 ************************************ */
#define AD2S1210_POSITION_0            0x80 // 位置高位（D15至D8）
#define AD2S1210_POSITION_1            0x81 // 位置低位（D7至D0）
#define AD2S1210_VELOCITY_0            0x82 // 速度高位（D15至D8）
#define AD2S1210_VELOCITY_1            0x83 // 速度低位（D7至D0）
#define AD2S1210_LOS_THRESHOLD         0x88 // LOS阈值
#define AD2S1210_DOS_OVER_THRESHOLD    0x89 // DOS超量程阈值
#define AD2S1210_DOS_MISS_THRESHOLD    0x8A // DOS失配阈值
#define AD2S1210_DOS_RES_MAX_THRESHOLD 0x8B // DOS复位最大阈值
#define AD2S1210_DOS_RES_MIN_THRESHOLD 0x8C // DOS复位最小阈值
#define AD2S1210_LOT_UPPER_LIMIT       0x8D // LOT上限
#define AD2S1210_LOT_LOWER_LIMIT       0X8E // LOT下限
#define AD2S1210_EXC_FRE               0X91 // 激励频率
#define AD2S1210_CONTROL               0X92 // 控制
#define AD2S1210_RESET                 0XF0 // 软复位
#define AD2S1210_FAULT                 0XFF // 故障

/********************************** 引脚定义 ************************************ */
#define A0_H1     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define A0_L1     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define A1_H1     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET)
#define A1_L1     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)
#define CS_H1     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET)
#define CS_L1     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET)
#define SMAPLE_H1 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)
#define SMAPLE_L1 HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)
#define RESET_H1  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET) // 新动力板子高复位
#define RESET_L1  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)
#define WR_H1     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET) // SPI1_CS
#define WR_L1     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)
#define DIR_H1    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define DIR_L1    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)

#define A0_H2     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define A0_L2     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define A1_H2     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)
#define A1_L2     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_H2     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET)
#define CS_L2     HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET)
#define SMAPLE_H2 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET)
#define SMAPLE_L2 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET)
#define RESET_H2  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET) // 新动力板子高复位
#define RESET_L2  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define WR_H2     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET) // SPI2_CS
#define WR_L2     HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET)
#define DIR_H2    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)
#define DIR_L2    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)

/********************************** 结构体定义 ************************************ */

typedef struct
{
    float Mechanical_Angle;
    float Electrical_Angle;
    float Electrical_Angle_offset;
    uint16_t Angle;

    uint8_t fluat_data;
    uint8_t Register_data;

    int16_t Speed_read;
    float Speed;
    float Current_Speed;

} ad2s1210_t;

typedef enum {
    POSITION,
    VELOCITY,
    CONFIG
} AD2S1210_CONTROL_MOD_ENUM;

typedef struct
{
    int position_data1;
    int position_data2;
    int velocity_data1;
    int velocity_data2;
    uint8_t fault1;
    uint8_t fault2;
} SPI_readresult;

typedef enum {
    ONE,
    TWO,
    ALL
} AD2S1210_CHIP_ENUM;

/********************************** 用户接口定义 ************************************ */

extern ad2s1210_t Load_AD2S;
extern ad2s1210_t Drive_AD2S;

/********************************** 函数定义 ************************************ */

void AD2S1210_Init(void);
void AD2S1210_ModeSelect(AD2S1210_CONTROL_MOD_ENUM mode); // 选择控制模式，普通模式or配置模式
void AD2S1210_ChipSelect(AD2S1210_CHIP_ENUM index);       // 选择哪个旋变，多个旋变时，自行增加代码内容
void AD2S1210_HW_RESET(void);                             // 硬件重启初始化

SPI_readresult AD2S1210_ReadPosition(AD2S1210_CHIP_ENUM index); // 使用前需要先确保A0A1模式匹配
SPI_readresult AD2S1210_ReadVelocity(AD2S1210_CHIP_ENUM index); // 使用前需要先确保A0A1模式匹配
SPI_readresult AD2S1210_ReadFault(void);                        // 使用前需要先确保A0A1模式匹配(普通模式)

unsigned char AD2S1210_ReadRegister(AD2S1210_CHIP_ENUM index, SPI_HandleTypeDef *hspi, unsigned char addr); // 使用前需要先确保A0A1模式匹配,配置模式
void AD2S1210_WriteRegister(SPI_HandleTypeDef *hspi, unsigned char addr, unsigned char data);               // 使用前需要先确保A0A1模式匹配,配置模式

unsigned char AD2S1210_GetFault(AD2S1210_CHIP_ENUM index, SPI_HandleTypeDef *hspi); // 故障发生时用于读取故障

void AD2S1210_para_Init(void);
void AD2S1210_Angle_Get(void);
void AD2S1210_Speed_Get(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_AD2S1210_H_ */