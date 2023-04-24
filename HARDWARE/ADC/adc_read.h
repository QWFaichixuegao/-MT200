#ifndef __ADC_READ_H_
#define __ADC_READ_H_

#include "adc.h"

typedef struct {
	__IO uint16_t  adc_value1;
	__IO uint16_t  adc_value2;
	float tem_c;
	float bat_v;
	
}READ_ADC;
extern READ_ADC read_adc;

void adc_intem_read(void);
void adc_batt_read(void);
#endif

