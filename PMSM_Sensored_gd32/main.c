//#include "stm32f10x.h"
#include "gd32f10x.h"
#include "gd32f10x_adc.h"
#include "gd32f10x_rcu.h"
#include "sysclk.h"
#include "adc_dma.h"
#include "pmsm.h"

#define TIME_CLOCK_SYSTICK 72000000

float InputPwmDutyCycle = 0, InputPwmFreq = 0;
uint8_t InputPWM_NoSignalFlag = 0, InputPWM_SignalRisingFlag = 0, InputPWM_SignalFallingFlag = 0;
float InputSignalPercentDutyCycle = 0;
float InputSignalFreq = 0;

extern uint8_t OverCurrentFlag;
void expRunningAverage(float newVal, float* filVal, float k);
float PWMSet = 0;
float Voltage, Current;
float Speed;
uint16_t CurrentZero;
extern volatile int8_t PMSM_Timing;
void SysTick_Handler(){
//	InputPWM_NoSignalFlag = 1;
//	InputSignalPercentDutyCycle = 0;
//	InputSignalFreq = 0;
}

///////////////////////////////////////////////////////////	 
////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
void EXTI15_10_IRQHandler(){

	if (RESET != exti_interrupt_flag_get(EXTI_12)) {		
		exti_interrupt_flag_clear(EXTI_12);		
		InputPWM_NoSignalFlag = 0;		
		if (gpio_input_bit_get(GPIOA, GPIO_PIN_12)) {			
			InputPwmFreq = (float)((uint32_t)SysTick_LOAD_RELOAD_Msk - (uint32_t)SysTick->VAL);
			InputPWM_SignalRisingFlag = 1;
//			InputSignalFreq = TIME_CLOCK_SYSTICK / InputPwmFreq;
			SysTick->VAL = SysTick_LOAD_RELOAD_Msk;
		} else {
			InputPwmDutyCycle = InputPwmFreq - ((uint32_t)SysTick_LOAD_RELOAD_Msk - (uint32_t)SysTick->VAL);
			InputPWM_SignalFallingFlag = 1;
//			InputSignalPercentDutyCycle = 100 - (float)(InputPwmDutyCycle * 100) / InputPwmFreq;
//			InputSignalPercentDutyCycle += 2;
		}	
	}
}

int main(void)
{
	SetSysClockTo72();
	
	// ADC Init
	ADC_DMA_init();
		
//	//PMSM Init
	PMSM_Init();

	CurrentZero = adc_channel_sample(ADC_CHANNEL_3);
	
//	rcu_periph_clock_enable(RCU_GPIOA);
//	gpio_init(GPIOA, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
//
//	/* enable the AF clock */
//	rcu_periph_clock_enable(RCU_AF);
//	/* enable and set key EXTI interrupt to the specified priority */
//	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
//	nvic_irq_enable(EXTI10_15_IRQn, 2U, 2U);
//
//	/* connect key EXTI line to key GPIO pin */
//	gpio_exti_source_select(GPIO_PORT_SOURCE_GPIOA, GPIO_PIN_SOURCE_12);
//
//	/* configure key EXTI line */
//	exti_init(EXTI_12, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
//	exti_interrupt_flag_clear(EXTI_12);
//	
	SysTick_Config(SysTick_LOAD_RELOAD_Msk);
	SysTick_CLKSourceConfig(SysTick_CTRL_CLKSOURCE_Msk);


    while(1)
    {	    
	    	    
	    Voltage = (float)adc_channel_sample(ADC_CHANNEL_0) * 0.00493767;
//	    expRunningAverage((float)ADCBuffer[1] * 0.0075 + 2.0924, &Current, 0.001);  //* 0.0008056640625 * 21.621
	    expRunningAverage((float)(TIME_CLOCK_SYSTICK / PMSM_GetSpeed()) / 22, &Speed, 0.001);	
	    
//	    if (Speed < 4500)
//		  PMSM_Timing = (Speed * 0.005);
	    
	    if (InputPWM_SignalFallingFlag)
	    {
		    InputPWM_SignalFallingFlag = 0;
		    InputSignalPercentDutyCycle = 100 - (float)(InputPwmDutyCycle * 100) / InputPwmFreq;		    
	    }
	    if (InputPWM_SignalRisingFlag)
	    {
		    InputPWM_SignalRisingFlag = 0;
		    InputSignalFreq = TIME_CLOCK_SYSTICK / InputPwmFreq;
	    }
	    	    
//#define CONTROL_ADC
#define CONTROL_PWM
	    
#if defined (CONTROL_ADC)
    	if (ADCBuffer[2] > PMSM_ADC_START) {    		
    		if (PMSM_MotorIsRun() == 0) {
    			// Start motor
    			PMSM_MotorSetSpin(PMSM_CW);
    			PMSM_MotorCommutation(PMSM_HallSensorsGetPosition());
    			PMSM_MotorSetRun();
    		}    		
//	       	PMSM_SetPWM(PMSM_ADCToPWM(ADCBuffer[2]));
	    	PMSM_SetPWM((uint16_t)PWMSet);
	    }else {
			PMSM_SetPWM(0);
	    }
#endif
	    
#if defined (CONTROL_PWM)
	    if(OverCurrentFlag)
	    {
		    while (1)
		    {
			    if ((InputSignalPercentDutyCycle > 90)||(InputSignalPercentDutyCycle < 10))
			    {
				    OverCurrentFlag = 0;	    
				    break;
			    }
		    }
	    }
	    else
	    {
		    if (PMSM_MotorIsRun() == 0)
		    {
			    if ((InputSignalPercentDutyCycle > 10) && (InputSignalPercentDutyCycle < 90)) {
				    // If Motor Is not run
				    if(PMSM_MotorIsRun() == 0) {
					    // Start motor
					    PMSM_MotorSetSpin(PMSM_CW);
					    PMSM_MotorCommutation(PMSM_HallSensorsGetPosition());
					    PMSM_MotorSetRun();
				    } else {				    
					    float PWM = (InputSignalPercentDutyCycle * 1.125 - 1.25) * ONE_PERCENT_DUTYCUCLE;				    
					    expRunningAverage(PWM, &PWMSet, 0.00003);
					    PMSM_SetPWM((uint16_t)PWMSet);
				    }
			    }
			    else
			    {
				    PMSM_MotorStop();			    
				    PWMSet = 0;
			    }		    
	    
		    }else
		    {
			    if ((InputSignalPercentDutyCycle > 5) && (InputSignalPercentDutyCycle < 95)) {
				    // If Motor Is not run
				    if(PMSM_MotorIsRun() == 0) {
					    // Start motor
					    PMSM_MotorSetSpin(PMSM_CW);
					    PMSM_MotorCommutation(PMSM_HallSensorsGetPosition());
					    PMSM_MotorSetRun();
				    } else {				    
					    float PWM = (InputSignalPercentDutyCycle * 1.125 - 1.25) * ONE_PERCENT_DUTYCUCLE;				    
					    expRunningAverage(PWM, &PWMSet, 0.00003);
					    PMSM_SetPWM((uint16_t)PWMSet);
				    }
			    }
			    else
			    {
				    PMSM_MotorStop();			    
				    PWMSet = 0;
			    }		    
			    
		    }

	    }
#endif
    }

}


// бегущее среднее
void expRunningAverage(float newVal, float* filVal, float k) {
	    // коэффициент фильтрации, 0.0-1.0	
	*filVal += (newVal - *filVal) * k;	
}


