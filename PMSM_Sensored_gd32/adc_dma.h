#include "gd32f10x_adc.h"
#include "gd32f10x_dma.h"
#include "gd32f10x_gpio.h"

volatile uint16_t ADCBuffer[3] = {0};

void ADC_DMA_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	/* enable ADC clock */
	rcu_periph_clock_enable(RCU_ADC0);
	/* enable DMA0 clock */
	rcu_periph_clock_enable(RCU_DMA0);
	/* config ADC clock */
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);
	
	gpio_init(GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_1);
	
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
	dma_data_parameter.number       = 3U;
	dma_data_parameter.priority     = DMA_PRIORITY_HIGH;
	//dma_init(DMA0, DMA_CH0, &dma_data_parameter);
	//dma_circulation_enable(DMA0, DMA_CH0);
  
	/* enable DMA channel */
	//dma_channel_enable(DMA0, DMA_CH0);
	
	
	adc_deinit(ADC0);
	/* ADC mode config */
	adc_mode_config(ADC_MODE_FREE);
	/* ADC contineous function enable */
	adc_special_function_config(ADC0, ADC_CONTINUOUS_MODE, DISABLE);
	/* ADC scan mode disable */
	adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
	/* ADC data alignment config */
	adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
	/* ADC channel length config */
	adc_channel_length_config(ADC0, ADC_REGULAR_CHANNEL, 1);

	/* ADC regular channel config */
	adc_regular_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
	adc_regular_channel_config(ADC0, 1, ADC_CHANNEL_1, ADC_SAMPLETIME_239POINT5);
	adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_2, ADC_SAMPLETIME_239POINT5);
	adc_regular_channel_config(ADC0, 3, ADC_CHANNEL_3, ADC_SAMPLETIME_239POINT5);
	//adc_regular_channel_config(ADC0, 2, ADC_CHANNEL_9, ADC_SAMPLETIME_239POINT5);
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
}

uint16_t adc_channel_sample(uint8_t channel)
{
	/* ADC regular channel config */
	adc_regular_channel_config(ADC0, 0U, channel, ADC_SAMPLETIME_7POINT5);
	/* ADC software trigger enable */
	adc_software_trigger_enable(ADC0, ADC_REGULAR_CHANNEL);

	/* wait the end of conversion flag */
	while (!adc_flag_get(ADC0, ADC_FLAG_EOC)) ;
	/* clear the end of conversion flag */
	adc_flag_clear(ADC0, ADC_FLAG_EOC);
	/* return regular channel sample value */
	return (adc_regular_data_read(ADC0));
}
//=================================================================================
