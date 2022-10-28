#include "gd32f10x_adc.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_gpio.h"

volatile uint16_t ADCBuffer[2] = {0};

void ADC_DMA_init(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	ADC_InitTypeDef ADC_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;

	rcu_periph_clock_enable(RCU_GPIOA);
	/* enable ADC clock */
	rcu_periph_clock_enable(RCU_ADC0);
	/* enable DMA0 clock */
	rcu_periph_clock_enable(RCU_DMA0);
	/* config ADC clock */
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
	
	gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 );
	
//	RCC_ADCCLKConfig(RCC_PCLK2_Div2);
    /* Enable ADC1 and GPIOA clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE );

	dma_parameter_struct dma_data_parameter;
    
	/* ADC DMA_channel configuration */
	dma_deinit(DMA0, DMA_CH0);
    
	/* initialize DMA single data mode */
	dma_data_parameter.periph_addr  = (uint32_t)(&ADC_RDATA(ADC0));
	dma_data_parameter.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	dma_data_parameter.memory_addr  = (uint32_t)(&ADCBuffer);
	dma_data_parameter.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	dma_data_parameter.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	dma_data_parameter.memory_width = DMA_MEMORY_WIDTH_16BIT;
	dma_data_parameter.direction    = DMA_PERIPHERAL_TO_MEMORY;
	dma_data_parameter.number       = 2U;
	dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
	dma_init(DMA0, DMA_CH0, &dma_data_parameter);
	dma_circulation_enable(DMA0, DMA_CH0);
  
	/* enable DMA channel */
	dma_channel_enable(DMA0, DMA_CH0);
	
//	DMA_InitStructure.DMA_BufferSize = 2;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADCBuffer;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
//	DMA_Cmd(DMA1_Channel1 , ENABLE );

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	adc_deinit(ADC0);
	/* ADC mode config */
	adc_mode_config(ADC_MODE_FREE);
	/* ADC contineous function enable */
	adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, ENABLE);
	/* ADC scan mode disable */
	adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
	/* ADC data alignment config */
	adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
	/* ADC channel length config */
	adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 2);

	/* ADC regular channel config */
	adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
	adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);

	/* ADC trigger config */
	adc_external_trigger_source_config(ADC0, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE);
	adc_external_trigger_config(ADC0, ADC_REGULAR_CHANNEL, ENABLE);
    
	/* enable ADC interface */
	adc_enable(ADC0);
	delay_1ms(1);
	/* ADC calibration and reset calibration */
	adc_calibration_enable(ADC0);
	/* ADC DMA function enable */
	adc_dma_mode_enable(ADC0);
	/* ADC software trigger enable */
	adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

//	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
//	ADC_InitStructure.ADC_NbrOfChannel = 2;
//	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
//	ADC_Init(ADC1, &ADC_InitStructure);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_239Cycles5);
////	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_239Cycles5);
//	ADC_Cmd(ADC1, ENABLE ) ;
//	ADC_DMACmd(ADC1, ENABLE ) ;
//	ADC_ResetCalibration(ADC1);
//
//	while(ADC_GetResetCalibrationStatus(ADC1));
//	ADC_StartCalibration(ADC1);
//
//	while(ADC_GetCalibrationStatus(ADC1));
//	ADC_SoftwareStartConvCmd ( ADC1 , ENABLE ) ;
}
//=================================================================================
