#include "adc_read.h"

READ_ADC read_adc;

// 读取主控温度
void adc_intem_read(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,50);
    read_adc.adc_value1 = HAL_ADC_GetValue(&hadc1);
    read_adc.tem_c = (1.43f - read_adc.adc_value1 * 3.3f / 4096) / 4.3f + 25;
}

// 读小电池电压
void adc_batt_read(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,50);
    read_adc.adc_value2 = HAL_ADC_GetValue(&hadc1);
    read_adc.bat_v = 4 * read_adc.adc_value2 * 3.3f / 4096;
}

