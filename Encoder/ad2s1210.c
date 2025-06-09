/*
 * AD2S1210.C
 *
 *  Created on: Dec 4, 2024
 *      Author: WML
 */
#include "ad2s1210.h"

ad2s1210_t Load_AD2S, Drive_AD2S;

/**
 * @brief   AD2S1210 延时
 * @param   time 延时循环次数
 */
static void AD2S1210_delay(unsigned int time) // 机械延时
{
    volatile unsigned int a = time;
    while (a > 0) a--;
}

/**
 * @brief   AD2S1210 使用定时器进行延时
 */
void delay_us(uint16_t us) // us延时
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
static unsigned char SPI_ReadByte(SPI_HandleTypeDef *hspi)
{
    unsigned char receivedByte = 0;
    // 发送数据并接收响应
    HAL_SPI_Receive(hspi, &receivedByte, 1, HAL_MAX_DELAY);
    // 返回接收到的数据
    return receivedByte;
}

/**
 * @brief   SPI 写字节函数
 */
static void SPI_WriteByte(SPI_HandleTypeDef *hspi, unsigned char buf)
{
    HAL_SPI_Transmit(hspi, &buf, 1, HAL_MAX_DELAY); // 使用 HAL 库发送数据
}

/**
 * @brief   AD2S1210 更新数据寄存器
 */
static void AD2S1210_UpdataRegister(void)
{
    SMAPLE_H1;
    SMAPLE_H2;
    SMAPLE_L1;
    SMAPLE_L2;
    AD2S1210_delay(10);
    SMAPLE_H1;
    SMAPLE_H2;
}

/**
 * @brief   AD2S1210 锁存数据(相当于SPI1_CS取消片选(1))
 */
static void AD2S1210_DataLock(void)
{
    WR_L1;
    WR_L2;
    AD2S1210_delay(1);
    WR_H1;
    WR_H2;
    AD2S1210_delay(1);
}

/**
 * @brief   AD2S1210 释放数据(相当于SPI1_CS片选(0))
 */
static void AD2S1210_DataRelease(void)
{
    WR_H1;
    WR_H2;
    AD2S1210_delay(1);
    WR_L1;
    WR_L2;
    AD2S1210_delay(1);
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
        A0_L1;
        A0_L2;
        A1_L1;
        A1_L2;
        AD2S1210_delay(1);
    } else if (mode == VELOCITY) {
        A0_L1;
        A0_L2;
        A1_H1;
        A1_H2;
        AD2S1210_delay(1);
    } else if (mode == CONFIG) {
        A0_H1;
        A0_H2;
        A1_H1;
        A1_H2;
        AD2S1210_delay(1);
    }
}

/**
 * @brief   AD2S1210 变量初始化
 */
void AD2S1210_para_Init(void)
{
    Load_AD2S.Mechanical_Angle        = 0;
    Load_AD2S.Electrical_Angle        = 0;
    Load_AD2S.Electrical_Angle_offset = 4.32;
    Load_AD2S.Angle                   = 0;
    Load_AD2S.fluat_data              = 0;
    Load_AD2S.Register_data           = 0;
    Load_AD2S.Speed                   = 0;
    Load_AD2S.Current_Angle           = 0;
    Load_AD2S.Last_Angle              = 0;
    Load_AD2S.angle_diff              = 0;
    Load_AD2S.Current_Speed           = 0;

    Drive_AD2S.Mechanical_Angle        = 0;
    Drive_AD2S.Electrical_Angle        = 0;
    Drive_AD2S.Electrical_Angle_offset = 2.96;
    Drive_AD2S.Angle                   = 0;
    Drive_AD2S.fluat_data              = 0;
    Drive_AD2S.Register_data           = 0;
    Drive_AD2S.Speed                   = 0;
    Drive_AD2S.Current_Angle           = 0;
    Drive_AD2S.Last_Angle              = 0;
    Drive_AD2S.angle_diff              = 0;
    Drive_AD2S.Current_Speed           = 0;
}

/**
 * @brief   AD2S1210 选择旋变芯片
 */
void AD2S1210_ChipSelect(AD2S1210_CHIP_ENUM index)
{
    switch (index) {
            //		case ONE:CS_L1;break;
        case ONE:
            CS_L1;
            CS_H2;
            break;
        case TWO:
            CS_H2;
            CS_L2;
            break;
        case ALL:
            CS_L1;
            CS_L2;
            break;
        default:
            break;
    }
    AD2S1210_delay(1);
}

/**
 * @brief   AD2S1210 硬件复位
 */
void AD2S1210_HW_RESET(void)
{
    RESET_H1;
    RESET_H2;
    SMAPLE_H1;
    SMAPLE_H2;
    HAL_Delay(10);
    RESET_L1;
    RESET_L2;
    HAL_Delay(80);
    SMAPLE_L1;
    SMAPLE_L2;
    HAL_Delay(1);
    SMAPLE_H1;
    SMAPLE_H2;
    // 复位结束
}

/*********************************Serial port mode (串口模式) ********************************** */
/**
 * @brief   AD2S1210 读取位置
 * @attention   由于故障发生概率较小，一般24位输出只读前面16位有效数据。当故障引脚变为低电平时进入配置模式读取故障。
 * @attention   使用前需要先确保A0A1模式匹配(普通模式)
 * @param   index   选择旋变编号
 */
SPI_readresult AD2S1210_ReadPosition(AD2S1210_CHIP_ENUM index)
{
    SPI_readresult result = {0};

    AD2S1210_ChipSelect(index); // 选旋变
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    /***SPI1***/
    result.position_data1 = SPI_ReadByte(&hspi1) << 8; // 高八位
    result.position_data1 += SPI_ReadByte(&hspi1);     // 低八位
    /***SPI2***/
    result.position_data2 = SPI_ReadByte(&hspi2) << 8; // 高八位
    result.position_data2 += SPI_ReadByte(&hspi2);     // 低八位

    //	AD2S.fluat_data = SPI_ReadByte();
    AD2S1210_DataLock();
    return result;
}

/**
 * @brief   AD2S1210 读取速度
 * @attention   由于故障发生概率较小，一般24位输出只读前面16位有效数据。当故障引脚变为低电平时进入配置模式读取故障。
 * @attention   使用前需要先确保A0A1模式匹配(普通模式)
 * @param   index   选择旋变编号
 */
SPI_readresult AD2S1210_ReadVelocity(AD2S1210_CHIP_ENUM index)
{
    SPI_readresult result = {0};
    AD2S1210_ChipSelect(index);
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    /***SPI1***/
    result.velocity_data1 = SPI_ReadByte(&hspi1) << 8;
    result.velocity_data1 += SPI_ReadByte(&hspi1);
    /***SPI2***/
    result.velocity_data2 = SPI_ReadByte(&hspi2) << 8; // 高八位
    result.velocity_data2 += SPI_ReadByte(&hspi2);     // 低八位

    AD2S1210_DataLock();

    return result; // 16位输出分辨率->65536
}

/**
 * @brief   AD2S1210 读取错误
 * @attention   由于故障发生概率较小，一般24位输出只读前面16位有效数据。当故障引脚变为低电平时进入配置模式读取故障。
 * @attention   使用前需要先确保A0A1模式匹配(普通模式)
 * @param   index   选择旋变编号
 */
SPI_readresult AD2S1210_ReadFault(void) // 使用前需要先确保A0A1模式匹配(普通模式)
{
    SPI_readresult result = {0};
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    /***SPI1***/
    SPI_ReadByte(&hspi1);
    SPI_ReadByte(&hspi1);
    result.fault1 = SPI_ReadByte(&hspi1);
    /***SPI2***/
    SPI_ReadByte(&hspi2);
    SPI_ReadByte(&hspi2);
    result.fault1 = SPI_ReadByte(&hspi2);

    AD2S1210_DataLock();

    return result;
}

/**
 * @brief   AD2S1210 配置模式读寄存器
 * @param   index   选择旋变编号
 * @param   addr    寄存器地址
 */
unsigned char AD2S1210_ReadRegister(AD2S1210_CHIP_ENUM index, SPI_HandleTypeDef *hspi, unsigned char addr)
{
    unsigned char buf = 0;
    AD2S1210_ModeSelect(CONFIG);
    AD2S1210_ChipSelect(index);
    AD2S1210_UpdataRegister();
    AD2S1210_DataRelease();
    SPI_WriteByte(hspi, addr);
    AD2S1210_DataLock();
    AD2S1210_delay(3);
    AD2S1210_DataRelease();
    buf = SPI_ReadByte(hspi);
    AD2S1210_DataLock();
    return buf;
}

/**
 * @brief   AD2S1210 配置模式写寄存器
 * @param   addr    寄存器地址
 * @param   data    写入数据
 */
void AD2S1210_WriteRegister(SPI_HandleTypeDef *hspi, unsigned char addr, unsigned char data)
{
    AD2S1210_ModeSelect(CONFIG);
    AD2S1210_DataRelease();
    SPI_WriteByte(hspi, addr);
    AD2S1210_DataLock();
    AD2S1210_delay(3);
    AD2S1210_DataRelease();
    SPI_WriteByte(hspi, data);
    AD2S1210_DataLock();
}

/**
 * @brief   AD2S1210 配置模式读故障寄存器
 * @param   index   选择旋变编号
 */
unsigned char AD2S1210_GetFault(AD2S1210_CHIP_ENUM index, SPI_HandleTypeDef *hspi) // 配置模式读故障寄存器
{
    AD2S1210_ChipSelect(index);
    AD2S1210_ModeSelect(CONFIG);
    return AD2S1210_ReadRegister(index, hspi, 0xFF); // 使用前需要先确保A0A1模式匹配,配置模式
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
    AD2S1210_ChipSelect(ALL);
    //	AD2S1210_WriteRegister(AD2S1210_CONTROL,0x7F);	//设置配置模式下，分辨率为16位

    /***SPI1配置***/
    AD2S1210_WriteRegister(&hspi1, AD2S1210_EXC_FRE, 32); // 设置激励信号为10kHZ（40），20=5k，80=20k
    AD2S1210_WriteRegister(&hspi1, AD2S1210_LOS_THRESHOLD, 0x01);
    AD2S1210_WriteRegister(&hspi1, AD2S1210_DOS_MISS_THRESHOLD, 0X7F);
    /***SPI2配置***/
    AD2S1210_WriteRegister(&hspi2, AD2S1210_EXC_FRE, 32); // 设置激励信号为10kHZ（40），20=5k，80=20k
    AD2S1210_WriteRegister(&hspi2, AD2S1210_LOS_THRESHOLD, 0x01);
    AD2S1210_WriteRegister(&hspi2, AD2S1210_DOS_MISS_THRESHOLD, 0X7F);

    //	AD2S.Register_data = AD2S1210_ReadRegister(ONE,AD2S1210_LOS_THRESHOLD);
    //  AD2S1210_ModeSelect(POSITION);
}

/**
 * @brief   AD2S1210 读取角度
 */
void AD2S1210_Angle_Get(void)
{
    AD2S1210_ModeSelect(POSITION);
    SPI_readresult res = AD2S1210_ReadPosition(ALL);
    /***电机1***/
    Drive_AD2S.Mechanical_Angle = (res.position_data1 - 32767) * M_PI / 32767.f;
    Drive_AD2S.Electrical_Angle = normalize(4, Drive_AD2S.Mechanical_Angle, Drive_AD2S.Electrical_Angle_offset);
    /***电机2***/
    Load_AD2S.Mechanical_Angle = (res.position_data2 - 32767) * M_PI / 32767.f;
    Load_AD2S.Electrical_Angle = normalize(-4, Load_AD2S.Mechanical_Angle, Load_AD2S.Electrical_Angle_offset);
}

/**
 * @brief   AD2S1210 读取电角速度
 */
void AD2S1210_Speed_Get(void)
{
    AD2S1210_ModeSelect(VELOCITY);
    SPI_readresult res = AD2S1210_ReadVelocity(ALL);
    /***电机1***/
    Drive_AD2S.Speed_read    = (int16_t)(res.velocity_data1);
    Drive_AD2S.Current_Speed = (float)(Drive_AD2S.Speed_read * 0.329f); // 125*60/2^15
    Drive_AD2S.Speed         = 0.01f * Drive_AD2S.Current_Speed + 0.99f * Drive_AD2S.Speed;

    /***电机2***/
    Load_AD2S.Speed_read    = (int16_t)(res.velocity_data2);
    Load_AD2S.Current_Speed = (float)(Load_AD2S.Speed_read * 0.329f); // 125*60/2^15
    Load_AD2S.Speed         = 0.01f * Load_AD2S.Current_Speed + 0.99f * Load_AD2S.Speed;
}