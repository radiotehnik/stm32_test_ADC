#include "main_utils.h"

//===============================================================================================================
//===============================��������� ������� 21 ��� ������� 2 � 3 ������===================================
//===============================================================================================================
void SetTimer21 (float T_period, uint16_t half_density) // TIMER21 - ��� ���������� � ����������� ������ ������.
{
	float t_dest = 0;
	
	uint16_t PSC_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	t_dest = T_period/(half_density*2); // �������� ����������� ������ �. � ��������
	
	ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(PSC_init+1)); // ��������� �������� ARR

	if ((ARR_opt_value_check-1) <= 65535)
	{
		TIM21->PSC = PSC_init;
		TIM21->ARR = (uint16_t)(ARR_opt_value_check-1);
	}
	
	else
	{
		for (uint16_t j = 1; j<=65535; j++) // ����� �������� PSC	 (������������ �� ������)
		{
			ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(j+1)); // ��������� �������� ARR
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
//===============================��������� ������� 21 ��� ������� 2 � 3 �����====================================
//===============================================================================================================


//===============================================================================================================
//===============================��������� ������� 21 ��� ������� 1 ������=======================================
//===============================================================================================================
void SetTimer21_Profile1 (uint8_t T_Acceleration) // TIMER21 - ��� ���������� � ����������� ������ ������.
{
	float t_dest = 0;
	
	uint16_t PSC_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	t_dest = T_Acceleration/(500.f); // �������� ����������� ������ �. � ��������
	
	ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(PSC_init+1)); // ��������� �������� ARR

	if ((ARR_opt_value_check-1) <= 65535)
	{
		TIM21->PSC = PSC_init;
		TIM21->ARR = (uint16_t)(ARR_opt_value_check-1);
	}
	
	else
	{
		for (uint16_t j = 1; j<=65535; j++) // ����� �������� PSC	 (������������ �� ������)
		{
			ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(j+1)); // ��������� �������� ARR
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
//===============================��������� ������� 21 ��� ������� 1 �����========================================
//===============================================================================================================




void Timer_Configure (float T_period, uint16_t half_density, uint16_t *psc_value,uint16_t *arr_value)
{
	float t_dest = 0;
	
	uint16_t PSC_init = 0;
	uint32_t ARR_opt_value_check = 0;
	
	t_dest = T_period/(half_density*2); // �������� ����������� ������ �. � ��������
	
	ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(PSC_init+1)); // ��������� �������� ARR

	if ((ARR_opt_value_check-1) <= 65535)
	{
		*psc_value = PSC_init;
		*arr_value = (uint16_t)(ARR_opt_value_check-1);
	}
	
	else
	{
		for (uint16_t j = 1; j<=65535; j++) // ����� �������� PSC	 (������������ �� ������)
		{
			ARR_opt_value_check = (uint32_t)((t_dest*TIMER21_INPUT_F)/(j+1)); // ��������� �������� ARR
			if ((ARR_opt_value_check-1) <= 65535)
			{
				*psc_value = j;
				*arr_value = (uint16_t)(ARR_opt_value_check-1);
				break;
			}
		}
	}
			
}






	

