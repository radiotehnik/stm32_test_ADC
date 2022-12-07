#include "main_utils.h"

//===============================================================================================================
//===============================Настройка таймера 21 для профиля 2 и 3 НАЧАЛО===================================
//===============================================================================================================
void SetTimer21 (float T_period, uint16_t half_density) // TIMER21 - для прерывания и подстановки нужных данных.
{
	float t_dest = 0;
	
	uint16_t PSC_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	t_dest = T_period/(half_density*2); // получаем необходимое дельта т. в Секундах
	
	ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(PSC_init+1)); // Определим значение ARR

	if ((ARR_opt_value_check-1) <= 65535)
	{
		TIM21->PSC = PSC_init;
		TIM21->ARR = (uint16_t)(ARR_opt_value_check-1);
	}
	
	else
	{
		for (uint16_t j = 1; j<=65535; j++) // Здесь переберём PSC	 (предделитель на таймер)
		{
			ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(j+1)); // Определим значение ARR
			if ((ARR_opt_value_check-1) <= 65535)
			{
				TIM21->PSC = j;
				TIM21->ARR = (uint16_t)(ARR_opt_value_check-1);
				break;
			}
		}
	}
			
}
//===============================================================================================================
//===============================Настройка таймера 21 для профиля 2 и 3 КОНЕЦ====================================
//===============================================================================================================


//===============================================================================================================
//===============================Настройка таймера 21 для профиля 1 НАЧАЛО=======================================
//===============================================================================================================
void SetTimer21_Profile1 (uint8_t T_Acceleration) // TIMER21 - для прерывания и подстановки нужных данных.
{
	float t_dest = 0;
	
	uint16_t PSC_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	t_dest = T_Acceleration/(500.f); // получаем необходимое дельта т. в Секундах
	
	ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(PSC_init+1)); // Определим значение ARR

	if ((ARR_opt_value_check-1) <= 65535)
	{
		TIM21->PSC = PSC_init;
		TIM21->ARR = (uint16_t)(ARR_opt_value_check-1);
	}
	
	else
	{
		for (uint16_t j = 1; j<=65535; j++) // Здесь переберём PSC	 (предделитель на таймер)
		{
			ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(j+1)); // Определим значение ARR
			if ((ARR_opt_value_check-1) <= 65535)
			{
				TIM21->PSC = j;
				TIM21->ARR = (uint16_t)(ARR_opt_value_check-1);
				break;
			}
		}
	}
			
}
//===============================================================================================================
//===============================Настройка таймера 21 для профиля 1 КОНЕЦ========================================
//===============================================================================================================




void Timer_Configure (float T_period, uint16_t half_density, uint16_t *psc_value,uint16_t *arr_value)
{
	float t_dest = 0;
	
	uint16_t PSC_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	t_dest = T_period/(half_density*2); // получаем необходимое дельта т. в Секундах
	
	ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(PSC_init+1)); // Определим значение ARR

	if ((ARR_opt_value_check-1) <= 65535)
	{
		*psc_value = PSC_init;
		*arr_value = (uint16_t)(ARR_opt_value_check-1);
	}
	
	else
	{
		for (uint16_t j = 1; j<=65535; j++) // Здесь переберём PSC	 (предделитель на таймер)
		{
			ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(j+1)); // Определим значение ARR
			if ((ARR_opt_value_check-1) <= 65535)
			{
				*psc_value = j;
				*arr_value = (uint16_t)(ARR_opt_value_check-1);
				break;
			}
		}
	}
			
}






	

