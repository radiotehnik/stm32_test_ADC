#ifndef __MAIN_UTILS_H
#define __MAIN_UTILS_H

#include "stm32l0xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h> 


#define TIMER21_INPUT_F 32000000.f


void Timer_Configure (float T_period, uint16_t half_density, uint16_t *psc_value,uint16_t *arr_value);
void SetTimer21 (float T_period, uint16_t half_density);
void SetTimer21_Profile1 (uint8_t T_Acceleration); // TIMER21 - для прерывания и подстановки нужных данных.



#endif /* __MAIN_UTILS_H */

