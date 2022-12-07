#include "Stepper_utils.h"
//======================================================================================================================================
//===================================������ �������� ��� ������� 2 � 3 ������===========================================================
//======================================================================================================================================
void Array_ARR_Calculation (uint16_t *ARR_arr,uint16_t *PSC_arr, uint16_t speedd) // ������� ��� ������������ ������� ARR � PSC
{
	float PWM_Frequency = 0;
	
	uint16_t PSC_opt_value_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
  for (uint16_t i = 0; i<=999; i++) // ���������� �� ������� PSC/ARR
  {	
		PWM_Frequency =((Sine_values_arr[i]*speedd)/60.f)*PULSE_REV; // �������� ��������� ������� � ��
		
		if (PWM_Frequency == 0) // ���� ��������� ������� = 0, �� ��������� �������������
		{
			*(ARR_arr+i) = 0;
			*(PSC_arr+i) = PSC_opt_value_init;
			continue;
		}
		else // ���� ��������� ������� �� 0, �� ��� �����
			
		{
			ARR_opt_value_check = (uint32_t)(F_PWM_INPUT/PWM_Frequency); // ��������� �������� ARR
			
			if ((ARR_opt_value_check-1) <= 65535)
			{
				*(ARR_arr+i) = (uint16_t)(ARR_opt_value_check-1);
				*(PSC_arr+i) = PSC_opt_value_init;	
			}
			
			else
			{
				// ������� ����������� �������� ��� ������������� (PSC) � �� ������� ������� ������ (ARR)
				for (uint16_t j = 1; j<=65535; j++) // ����� �������� PSC	 (������������ �� ������)
				{
					ARR_opt_value_check = (uint32_t)((F_PWM_INPUT/(j+1))/PWM_Frequency);
					if ((ARR_opt_value_check-1) <= 65535) // ���� �� ���������� ������������, �� ���������� ��� ��������
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
//===================================������ �������� ��� ������� 2 � 3 �����============================================================
//======================================================================================================================================


//======================================================================================================================================
//===================================������ �������� ��� ������� 1 ������===============================================================
//======================================================================================================================================
void Array_Calculation_profile1 (uint16_t *ARR_arr,uint16_t *PSC_arr, uint16_t speedd) // ������� ��� ������������ ������� ARR � PSC
{
	float PWM_Frequency = 0;
	
	uint16_t PSC_opt_value_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	PWM_Frequency =((speedd)/60.f)*PULSE_REV; // �������� ��������� ������������ ������� � ��
	float delta_PWM_Frequency = PWM_Frequency/500;
	PWM_Frequency = 0;
	
  for (uint16_t i = 0; i<=499; i++) // ���������� �� ������� PSC/ARR
  {	
		PWM_Frequency+= delta_PWM_Frequency;
		
		if (PWM_Frequency == 0) // ���� ��������� ������� = 0, �� ��������� �������������
		{
			*(ARR_arr+i) = 0;
			*(PSC_arr+i) = PSC_opt_value_init;
			continue;
		}
		else // ���� ��������� ������� �� 0, �� ��� �����
			
		{
			ARR_opt_value_check = (uint32_t)(F_PWM_INPUT/PWM_Frequency); // ��������� �������� ARR
			
			if ((ARR_opt_value_check-1) <= 65535)
			{
				*(ARR_arr+i) = (uint16_t)(ARR_opt_value_check-1);
				*(PSC_arr+i) = PSC_opt_value_init;	
			}
			
			else
			{
				// ������� ����������� �������� ��� ������������� (PSC) � �� ������� ������� ������ (ARR)
				for (uint16_t j = 1; j<=65535; j++) // ����� �������� PSC	 (������������ �� ������)
				{
					ARR_opt_value_check = (uint32_t)((F_PWM_INPUT/(j+1))/PWM_Frequency);
					if ((ARR_opt_value_check-1) <= 65535) // ���� �� ���������� ������������, �� ���������� ��� ��������
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
//===================================������ �������� ��� ������� 1 �����================================================================
//======================================================================================================================================





void Forvard_run()
{
	HAL_GPIO_WritePin (GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
	HAL_Delay(1000);
	
	for(int i = maxF_; i >= minF_; i-=step_) // ����������� �������
  {		
	TIM2->ARR = i; // �� ������� ������� �������
  TIM2->CCR1 = i/2;
	HAL_Delay(delay_value_);
  }
						
HAL_Delay(5000);

	for(int i = minF_; i <= maxF_; i+=step_) // ��������� �������
	{
		TIM2->ARR = i; // �� ������� ������� �������
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
	
	for(int i = maxF_; i >= minF_; i-=step_) // ����������� �������
  {		
	TIM2->ARR = i; // �� ������� ������� �������
  TIM2->CCR1 = i/2;
	HAL_Delay(delay_value_);
  }
						
HAL_Delay(5000);

	for(int i = minF_; i <= maxF_; i+=step_) // ��������� �������
	{
		TIM2->ARR = i; // �� ������� ������� �������
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
