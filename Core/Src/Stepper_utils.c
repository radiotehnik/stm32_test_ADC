#include "Stepper_utils.h"
//======================================================================================================================================
//===================================Расчёт массивов для профиля 2 и 3 НАЧАЛО===========================================================
//======================================================================================================================================
void Array_ARR_Calculation (uint16_t *ARR_arr,uint16_t *PSC_arr, uint16_t speedd) // Функция для формирования массива ARR и PSC
{
	float PWM_Frequency = 0;
	
	uint16_t PSC_opt_value_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
  for (uint16_t i = 0; i<=999; i++) // проходимся по массиву PSC/ARR
  {	
		PWM_Frequency =((Sine_values_arr[i]*speedd)/60.f)*PULSE_REV; // получаем требуемую частоту в Гц
		
		if (PWM_Frequency == 0) // Если требуемая частота = 0, то двигатель останавливаем
		{
			*(ARR_arr+i) = 0;
			*(PSC_arr+i) = PSC_opt_value_init;
			continue;
		}
		else // Если требуемая частота НЕ 0, то идём далее
			
		{
			ARR_opt_value_check = (uint32_t)(F_PWM_INPUT/PWM_Frequency); // Определим значение ARR
			
			if ((ARR_opt_value_check-1) <= 65535)
			{
				*(ARR_arr+i) = (uint16_t)(ARR_opt_value_check-1);
				*(PSC_arr+i) = PSC_opt_value_init;	
			}
			
			else
			{
				// Подберём оптимальные значения для предллелителя (PSC) и до скольки считает таймер (ARR)
				for (uint16_t j = 1; j<=65535; j++) // Здесь переберём PSC	 (предделитель на таймер)
				{
					ARR_opt_value_check = (uint32_t)((F_PWM_INPUT/(j+1))/PWM_Frequency);
					if ((ARR_opt_value_check-1) <= 65535) // Если не происходит переполнения, то используем это значение
					{
						*(PSC_arr+i) = j;
						*(ARR_arr+i) = (uint16_t)(ARR_opt_value_check-1);
						break;
					}	
				}
				
			}
		}		
  }
	

}
//======================================================================================================================================
//===================================Расчёт массивов для профиля 2 и 3 КОНЕЦ============================================================
//======================================================================================================================================


//======================================================================================================================================
//===================================Расчёт массивов для профиля 1 НАЧАЛО===============================================================
//======================================================================================================================================
void Array_Calculation_profile1 (uint16_t *ARR_arr,uint16_t *PSC_arr, uint16_t speedd) // Функция для формирования массива ARR и PSC
{
	float PWM_Frequency = 0;
	
	uint16_t PSC_opt_value_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	PWM_Frequency =((speedd)/60.f)*PULSE_REV; // получаем требуемую максимальную частоту в Гц
	float delta_PWM_Frequency = PWM_Frequency/500;
	PWM_Frequency = 0;
	
  for (uint16_t i = 0; i<=499; i++) // проходимся по массиву PSC/ARR
  {	
		PWM_Frequency+= delta_PWM_Frequency;
		
		if (PWM_Frequency == 0) // Если требуемая частота = 0, то двигатель останавливаем
		{
			*(ARR_arr+i) = 0;
			*(PSC_arr+i) = PSC_opt_value_init;
			continue;
		}
		else // Если требуемая частота НЕ 0, то идём далее
			
		{
			ARR_opt_value_check = (uint32_t)(F_PWM_INPUT/PWM_Frequency); // Определим значение ARR
			
			if ((ARR_opt_value_check-1) <= 65535)
			{
				*(ARR_arr+i) = (uint16_t)(ARR_opt_value_check-1);
				*(PSC_arr+i) = PSC_opt_value_init;	
			}
			
			else
			{
				// Подберём оптимальные значения для предллелителя (PSC) и до скольки считает таймер (ARR)
				for (uint16_t j = 1; j<=65535; j++) // Здесь переберём PSC	 (предделитель на таймер)
				{
					ARR_opt_value_check = (uint32_t)((F_PWM_INPUT/(j+1))/PWM_Frequency);
					if ((ARR_opt_value_check-1) <= 65535) // Если не происходит переполнения, то используем это значение
					{
						*(PSC_arr+i) = j;
						*(ARR_arr+i) = (uint16_t)(ARR_opt_value_check-1);
						break;
					}	
				}
				
			}
		}		
  }
	

}
//======================================================================================================================================
//===================================Расчёт массивов для профиля 1 КОНЕЦ================================================================
//======================================================================================================================================





void Forvard_run()
{
	HAL_GPIO_WritePin (GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_Delay(1000);
	
	for(int i = maxF_; i >= minF_; i-=step_) // Увеличиваем частоту
  {		
	TIM2->ARR = i; // до скольки считает регистр
  TIM2->CCR1 = i/2;
	HAL_Delay(delay_value_);
  }
						
HAL_Delay(5000);

	for(int i = minF_; i <= maxF_; i+=step_) // Уменьшаем частоту
	{
		TIM2->ARR = i; // до скольки считает регистр
		TIM2->CCR1 = i/2;
		HAL_Delay(delay_value_);
		//TIM2->PSC = 65535;
	}
HAL_Delay(5000);
TIM2->CCR1 = 0;

}

void Back_run()
{
	HAL_GPIO_WritePin (GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
	HAL_Delay(1000);
	
	for(int i = maxF_; i >= minF_; i-=step_) // Увеличиваем частоту
  {		
	TIM2->ARR = i; // до скольки считает регистр
  TIM2->CCR1 = i/2;
	HAL_Delay(delay_value_);
  }
						
HAL_Delay(5000);

	for(int i = minF_; i <= maxF_; i+=step_) // Уменьшаем частоту
	{
		TIM2->ARR = i; // до скольки считает регистр
		TIM2->CCR1 = i/2;
		HAL_Delay(delay_value_);
									
	}
HAL_Delay(5000);
TIM2->CCR1 = 0;
}

uint16_t PeriodToDegreesNormalization (int x, int in_min, int in_max, int out_min, int out_max)
{
	  return (((x - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min;
}
