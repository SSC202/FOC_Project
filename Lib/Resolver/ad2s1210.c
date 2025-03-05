/*
 * AD2S1210.C
 *
 *  Created on: Dec 4, 2024
 *      Author: WML
 */
#include "ad2s1210.h"

ad2s1210_t Load_AD2S;

/**
 * @brief   AD2S1210 延时
 * @param   time 延时循环次数
 */
static void AD2S1210_delay(unsigned int time)
{
    volatile unsigned int a = time;
    while (a > 0) a--;
}

/**
 * @brief   AD2S1210 使用定时器进行延时
 */
static void delay_us(uint16_t us) // us延时
{
    // uint16_t differ = 0xffff - us - 5;
    // HAL_TIM_Base_Start(&htim7);
    // __HAL_TIM_SetCounter(&htim7, differ);
    // while (differ < 0xffff - 5) {
    //     differ = __HAL_TIM_GetCounter(&htim7);
    // }
    // HAL_TIM_Base_Stop(&htim7);
}

// SPI 函数(用户只需要修改硬件SPI函数即可，使用软件SPI不需要动具体函数)
/**
 * @brief   SPI 读字节函数
 */
static unsigned char SPI_ReadByte(void)
{
    unsigned char receivedByte = 0;
    // 发送数据并接收响应
    HAL_SPI_Receive(&hspi1, &receivedByte, 1, HAL_MAX_DELAY);
    // 返回接收到的数据
    return receivedByte;
}

/**
 * @brief   SPI 写字节函数
 */
static void SPI_WriteByte(unsigned char buf)
{
    HAL_SPI_Transmit(&hspi1, &buf, 1, HAL_MAX_DELAY); // 使用 HAL 库发送数据
}

/**
 * @brief   AD2S1210 更新数据寄存器
 */
static void AD2S1210_UpdataRegister(void)
{
    SMAPLE_H;
    SMAPLE_L;
    AD2S1210_delay(10);
    SMAPLE_H;
}

/**
 * @brief   AD2S1210 锁存数据(相当于SPI1_CS取消片选(1))
 */
static void AD2S1210_DataLock(void)
{
    WR_L;
    AD2S1210_delay(2);
    WR_H;
    AD2S1210_delay(2);
}

/**
 * @brief   AD2S1210 释放数据(相当于SPI1_CS片选(0))
 */
static void AD2S1210_DataRelease(void)
{
    WR_H;
    AD2S1210_delay(2);
    WR_L;
    AD2S1210_delay(2);
}

/********************************* 中间层函数段 ********************************** */

/**
 * @brief   AD2S1210 模式选择
 * @param   mode 选择的模式
 *          POSITION:   位置模式
 *          VELOCITY:   速度模式
 *          CONFIG:     配置模式
 */
void AD2S1210_ModeSelect(AD2S1210_CONTROL_MOD_ENUM mode)
{
    if (mode == POSITION) {
        A0_L;
        A1_L;
        AD2S1210_delay(2);
    } else if (mode == VELOCITY) {
        A0_L;
        A1_H;
        AD2S1210_delay(2);
    } else if (mode == CONFIG) {
        A0_H;
        A1_H;
        AD2S1210_delay(2);
    }
}

/**
 * @brief   AD2S1210 变量初始化
 */
void AD2S1210_para_Init(void)
{
    Load_AD2S.Mechanical_Angle        = 0;
    Load_AD2S.Electrical_Angle        = 0;
    Load_AD2S.Electrical_Angle_offset = -3.26f;
    Load_AD2S.Angle                   = 0;
    Load_AD2S.fluat_data              = 0;
    Load_AD2S.Register_data           = 0;
    Load_AD2S.Speed                   = 0;
    Load_AD2S.Current_Angle           = 0;
    Load_AD2S.Last_Angle              = 0;
    Load_AD2S.angle_diff              = 0;
    Load_AD2S.Current_Speed           = 0;
}

/**
 * @brief   AD2S1210 选择旋变芯片
 */
void AD2S1210_ChipSelect(AD2S1210_CHIP_ENUM index)
{
    switch (index) {
        case ONE:
            CS1_L;
            break;
            //		case ONE:CS1_L;CS2_H;break;
            //		case TWO:CS1_H;CS2_L;break;
            //		case ALL:CS1_L;CS2_L;break;
        default:
            break;
    }
    AD2S1210_delay(2);
}

/**
 * @brief   AD2S1210 硬件复位
 */
void AD2S1210_HW_RESET(void)
{
    RESET_H;
    SMAPLE_H;
    HAL_Delay(1);
    RESET_L;
    HAL_Delay(80);
    SMAPLE_L;
    HAL_Delay(1);
    SMAPLE_H;
    // 复位结束
}

/*********************************Serial port mode (串口模式) ********************************** */
/**
 * @brief   AD2S1210 读取位置
 * @attention   由于故障发生概率较小，一般24位输出只读前面16位有效数据。当故障引脚变为低电平时进入配置模式读取故障。
 * @attention   使用前需要先确保A0A1模式匹配(普通模式)
 * @param   index   选择旋变编号
 */
int AD2S1210_ReadPosition(AD2S1210_CHIP_ENUM index)
{
    int position_data;
    AD2S1210_ChipSelect(index); // 选旋变
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    position_data = SPI_ReadByte() << 8; // 高八位
    position_data += SPI_ReadByte();     // 低八位
    //	Load_AD2S.fluat_data = SPI_ReadByte();
    AD2S1210_DataLock();
    return position_data;
}

/**
 * @brief   AD2S1210 读取速度
 * @attention   由于故障发生概率较小，一般24位输出只读前面16位有效数据。当故障引脚变为低电平时进入配置模式读取故障。
 * @attention   使用前需要先确保A0A1模式匹配(普通模式)
 * @param   index   选择旋变编号
 */
int AD2S1210_ReadVelocity(AD2S1210_CHIP_ENUM index)
{
    int velocity_data = 0;
    AD2S1210_ChipSelect(index);
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    velocity_data = SPI_ReadByte() << 8;
    velocity_data += SPI_ReadByte();
    AD2S1210_DataLock();
    return velocity_data; // 16位输出分辨率->65536
}

/**
 * @brief   AD2S1210 读取错误
 * @attention   由于故障发生概率较小，一般24位输出只读前面16位有效数据。当故障引脚变为低电平时进入配置模式读取故障。
 * @attention   使用前需要先确保A0A1模式匹配(普通模式)
 * @param   index   选择旋变编号
 */
unsigned char AD2S1210_ReadFault(void) // 使用前需要先确保A0A1模式匹配(普通模式)
{
    unsigned char fault = 0;
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    SPI_ReadByte();
    SPI_ReadByte();
    fault = SPI_ReadByte();
    AD2S1210_DataLock();
    return fault;
}

/**
 * @brief   AD2S1210 配置模式读寄存器
 * @param   index   选择旋变编号
 * @param   addr    寄存器地址
 */
unsigned char AD2S1210_ReadRegister(AD2S1210_CHIP_ENUM index, unsigned char addr)
{
    unsigned char buf = 0;
    AD2S1210_ModeSelect(CONFIG);
    AD2S1210_ChipSelect(index);
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    SPI_WriteByte(addr);
    AD2S1210_DataLock();
    AD2S1210_delay(3);
    AD2S1210_DataRelease();
    buf = SPI_ReadByte();
    AD2S1210_DataLock();
    return buf;
}

/**
 * @brief   AD2S1210 配置模式写寄存器
 * @param   addr    寄存器地址
 * @param   data    写入数据
 */
void AD2S1210_WriteRegister(unsigned char addr, unsigned char data)
{
    AD2S1210_ModeSelect(CONFIG);
    AD2S1210_DataRelease();
    SPI_WriteByte(addr);
    AD2S1210_DataLock();
    AD2S1210_delay(3);
    AD2S1210_DataRelease();
    SPI_WriteByte(data);
    AD2S1210_DataLock();
}

/**
 * @brief   AD2S1210 配置模式读故障寄存器
 * @param   index   选择旋变编号
 */
unsigned char AD2S1210_GetFault(AD2S1210_CHIP_ENUM index) // 配置模式读故障寄存器
{
    AD2S1210_ChipSelect(index);
    AD2S1210_ModeSelect(CONFIG);
    return AD2S1210_ReadRegister(index, 0xFF);
}

/********************************* 用户函数段 ********************************** */

/**
 * @brief   AD2S1210 用户初始化
 */
void AD2S1210_Init(void)
{
    AD2S1210_para_Init();
    // SPI
    AD2S1210_HW_RESET();
    AD2S1210_DataLock();
    AD2S1210_ModeSelect(CONFIG);
    AD2S1210_ChipSelect(ONE);
    //	AD2S1210_WriteRegister(AD2S1210_CONTROL,0x7F);	//设置配置模式下，分辨率为16位
    AD2S1210_WriteRegister(AD2S1210_EXC_FRE, 32); // 设置激励信号为10kHZ（40），20=5k，80=20k
    AD2S1210_WriteRegister(AD2S1210_LOS_THRESHOLD, 0x01);
    AD2S1210_WriteRegister(AD2S1210_DOS_MISS_THRESHOLD, 0X7F);
    //	Load_AD2S.Register_data = AD2S1210_ReadRegister(ONE,AD2S1210_LOS_THRESHOLD);
    AD2S1210_ModeSelect(POSITION);
}

/**
 * @brief   AD2S1210 读取角度
 */
void AD2S1210_Angle_Get(void)
{
    Load_AD2S.Mechanical_Angle = (AD2S1210_ReadPosition(ONE) - 32767) * M_PI / 32767.f;
    Load_AD2S.Electrical_Angle = normalize(4, Load_AD2S.Mechanical_Angle, Load_AD2S.Electrical_Angle_offset);
}

/**
 * @brief   AD2S1210 读取速度
 */
void AD2S1210_Speed_Get(float t_sample)
{
    Load_AD2S.Current_Angle = Load_AD2S.Electrical_Angle;
    Load_AD2S.angle_diff    = (Load_AD2S.Current_Angle - Load_AD2S.Last_Angle);
    if (Load_AD2S.angle_diff > M_PI) {
        Load_AD2S.angle_diff = Load_AD2S.angle_diff - 2 * M_PI;
    } else if (Load_AD2S.angle_diff < -M_PI) {
        Load_AD2S.angle_diff = Load_AD2S.angle_diff + 2 * M_PI;
    } else {
        Load_AD2S.angle_diff = Load_AD2S.angle_diff;
    }

    Load_AD2S.Speed      = Load_AD2S.angle_diff / t_sample;
    Load_AD2S.Last_Angle = Load_AD2S.Current_Angle;
}
