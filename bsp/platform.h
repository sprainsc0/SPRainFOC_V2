#ifndef PLATFORM_H
#define PLATFORM_H

#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include "cmsis_os.h"

#include "tim.h"
#include "adc.h"
#include "gpio.h"

#define ADC_CURRENT_OHM		                  0.002f
#define ADC_CURRENT_AMP		                  20.0f
#define VREFINT                               1.2f
#define VREF                                  3.3f

#define RESISTANCE1                           39.0f
#define RESISTANCE2                           2.2f

void platform_init(void);
void hal_pwm_duty_write(uint32_t duty_a, uint32_t duty_b, uint32_t duty_c);
void hal_update_samp(uint32_t samp);

void pwm_output_on(void);
void pwm_output_off(void);

void pwm_output_ch1(void);
void pwm_output_ch2(void);
void pwm_output_ch3(void);

void pwm_forced_ch1(void);
void pwm_forced_ch2(void);
void pwm_forced_ch3(void);

void gate_on(void);
void gate_off(void);

int8_t drv_fault(void);

void current_filter_on(void);
void current_filter_off(void);

void pwm_gpio_forced_off(void);
void pwm_gpio_forced_on(void);

uint8_t fault_pin(void);

uint8_t calculate_deadtime(float deadtime_ns, float core_clock_freq);

#ifdef __cplusplus
}
#endif

#endif
