#include "hwdac.h"

static uint8_t ch_mapping[8] = {1, 2, 3, 4, 5, 6, 7, 8}; // 通道映射（根据实际硬件连接顺序）

uint16_t hwdac_value1; // 外部DAC1通道值
uint16_t hwdac_value2; // 外部DAC2通道值
uint16_t hwdac_value3; // 外部DAC3通道值
uint16_t hwdac_value4; // 外部DAC4通道值

/***********************************硬件函数***************************************** */

/**
 * @brief   AD5676 延时函数
 */
static void AD5676_delay(unsigned int time) // 机械延时
{
    volatile unsigned int a = time;
    while (a > 0) a--;
}

/**
 * @brief   AD5676 软件复位函数
 */
static void AD5676_reset(void)
{
    DAC_RESET_L;
    AD5676_delay(1);
    DAC_RESET_H;
    AD5676_delay(1);
}

/**
 * @brief   AD5676 触发所有通道输出函数
 */
static void AD5676_update(void)
{
    DAC_LDAC_L;
    AD5676_delay(1);
    DAC_LDAC_H;
}

/**
 * @brief   AD5676 触发通道上电函数
 */
static HAL_StatusTypeDef AD5676_power_up(uint8_t ch)
{
    HAL_StatusTypeDef ret;
    uint16_t mask = 0x0000;
    uint8_t data[3];

    if (ch < 1 || ch > 8) return HAL_ERROR;

    ch -= 1;
    mask &= ~(0x03 << (ch * 2)); // Power-up the specific DAC channel

    DAC_SYNC_L;
    data[0] = 0x40; // Power control command
    data[1] = mask >> 8;
    data[2] = mask & 0xFF;
    ret     = HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);
    DAC_SYNC_H;
    AD5676_update();
    return ret;
}

/***********************************用户函数***************************************** */

/**
 * @brief   AD5676 初始化函数
 */
void AD5676_init(void)
{
    DAC_GAIN_H;
    AD5676_reset();
    for (int i = 1; i <= 8; i++) {
        AD5676_power_up(i);
    }
}

/**
 * @brief   AD5676 设置某通道电压值
 * @param   ch      通道号
 * @param   value   DAC数值(0-65535对应0-5V)
 */
static HAL_StatusTypeDef AD5676_set_value(uint8_t ch, uint16_t value)
{
    HAL_StatusTypeDef ret;
    uint8_t data[3];

    if (ch < 1 || ch > 8) return HAL_ERROR;

    DAC_SYNC_L;
    data[0] = 0x30 | ((ch_mapping[ch - 1] - 1) & 0x0F); // 0x30 = 写并更新
    data[1] = value >> 8;
    data[2] = value & 0xFF;
    ret     = HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);
    DAC_SYNC_H;
    AD5676_update();
    return ret;
}

/**
 * @brief       AD5676 输出
 * @attention   设置外部ADC1-4的通道值
 */
void hw_dac_output(void)
{
	AD5676_set_value(1, hwdac_value1);
	AD5676_set_value(2, hwdac_value2);
	AD5676_set_value(3, hwdac_value3);
  AD5676_set_value(4, hwdac_value4);
}
