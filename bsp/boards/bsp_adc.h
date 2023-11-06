#ifndef BSP_ADC_H
#define BSP_ADC_H
#include "struct_typedef.h"

extern void bsp_adc_init_vrefint_reciprocal(void);
extern fp32 bsp_adc_get_temperature(void);
extern fp32 bsp_adc_get_battery_voltage(void);
extern uint8_t bsp_adc_get_hardware_version(void);
#endif
