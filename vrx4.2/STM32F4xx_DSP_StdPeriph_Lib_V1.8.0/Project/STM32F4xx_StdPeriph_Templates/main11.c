/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/** @addtogroup Template_Project
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t uwTimingDelay = 0;
//volatile adc_buffer_t adcfifo;
volatile adc_buffer_t adcfifo;
uint32_t maxpowerindex,maxpowerindex1,mm=0;
float32_t fft_source_data[FFT_N*2];
//float32_t fft_source_data[1024*2];
static float32_t resultOutput[ FFT_N];
char comp[21]={0x55,0xab,0x57,0x12,0x0c,0xa8,0xd1,0xff,0x00,0xcc,0x89,0x11,0xef,0x88,0x22,0xa};
uint32_t testIndex,testIndex0,testIndex1,value=0,valueback=0,m,error;
float32_t maxValue,maxpower;
/* Private function prototypes -----------------------------------------------*/
static void Delay(__IO uint32_t nTime);
static void TIM2_Config(void);
static void TIM3_Config(void);
static void ADC_Config(void);
static void GPIO_Config(void);
static void USART_Config(void);
void ADC_FifoInit(void);
void fft(void);
void seek( uint8_t direction,uint32_t lng);
void readfifo(uint32_t size);
static void Delay(__IO uint32_t nTime);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	uint32_t cnt;
	GPIO_Config();
	USART_Config();
	TIM3_Config();
	//TIM2_Config();
	ADC_Config();     
	/* Start ADC1 Software Conversion */ 
 // ADC_SoftwareStartConv(ADC1);
	GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_SET);
  /* Infinite loop */

  	printf("\nSystem clock rate: %d Hz\r\n", SystemCoreClock);
	mm=0;
	//ADC_FifoInit();	
	printf("Search sync ......");
bitsync:
	cnt=0;
	maxpower=0;
	maxpowerindex=0;
	ADC_FifoInit();
	//GPIO_SetBits(GPIOA,GPIO_Pin_1);
	while(1)
	{			
		readfifo(FFT_N);fft();	
		if(testIndex ==19)				
		{				
			if(maxValue > maxpower) {maxpower = maxValue; maxpowerindex=adcfifo.readptr;}
		}						
		cnt++;
		if(cnt>=128)
		{
			//seek(BACKWARD,adcfifo.readptr-maxpowerindex);
			break;
		}
		seek(FORWARD,2);
	}

	seek(BACKWARD,adcfifo.readptr-maxpowerindex);
	seek(FORWARD,FFT_N);
	cnt=0;value=0;
	value =0;m=0;mm=0;error=0;
	while(1)
	{
		readfifo(FFT_N); fft();
		if((testIndex >=3 && testIndex<=4))
 	  {/*printf("1")*//*GPIO_SetBits(GPIOB,GPIO_Pin_5);*/ value |= (1<<(7-m));m++;}
		else if( testIndex>=11 && testIndex<=13)
    { /*GPIO_ResetBits(GPIOB,GPIO_Pin_5);*//*printf("0");*/m++;}
		else 
		{ 
		}
		if(m==8) 
		{
			//if(value!=0xaa && value!= 0x7e) 
			{
				printf("0x%02x,",value);
				comp[mm]=value;
				if(value - valueback !=1 )
					GPIO_ResetBits(GPIOA,GPIO_Pin_1);
				else
					GPIO_SetBits(GPIOA,GPIO_Pin_1);
				valueback =value;
				
			}
			goto bitsync;
			
		}
		seek(FORWARD,FFT_N);
	}
  while (1)
  {
   //  printf("ok\r\n");
  }
}


static void ADC_Config(void)
{
  ADC_InitTypeDef       	ADC_InitStructure;
  ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
  GPIO_InitTypeDef       	GPIO_InitStructure;

 
    
  /* Enable peripheral clocks *************************************************/
  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


	  /* Configure ADC Channel 12 pin as analog input *****************************/ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  //20Hz
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;  //AGC_IN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
    
  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  
  /* ADC1 regular channel18 (VBAT) configuration ******************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_15Cycles);

	//ADC_DiscModeChannelCountConfig(ADC1,1);
	//ADC_DiscModeCmd(ADC1,ENABLE);
	ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);

  /* Enable ADC1 **************************************************************/
	NVIC_EnableIRQ(ADC_IRQn);
  ADC_Cmd(ADC1, ENABLE);
}




static void TIM3_Config(void)
{
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
//  NVIC_InitTypeDef NVIC_InitStruct;
	uint32_t uwPrescalerValue = 0;
  /* --------------------------------------------------------
  TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1), 
  since APB1 prescaler is different from 1.   
    TIM2CLK = 2 * PCLK1  
    TIM2CLK = HCLK / 2 = SystemCoreClock /2 

  Note: 
   SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
   Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
   function to update SystemCoreClock variable value. Otherwise, any configuration
   based on this variable will be incorrect.    

  ----------------------------------------------------------- */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 100000) - 1);   //TIM2CLK=100K
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period =  156; // 78-1;      //156-1    
  TIM_TimeBaseStructure.TIM_Prescaler = uwPrescalerValue;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* TIM2 TRGO selection */
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	/*
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	*/
  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_EnableIRQ(TIM3_IRQn);
  TIM_Cmd(TIM3, ENABLE);
}
static void TIM2_Config(void)
{
  TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	TIM_OCInitTypeDef  TIM_OCInitStructure;

  NVIC_InitTypeDef NVIC_InitStruct;
	uint32_t uwPrescalerValue = 0;
  /* --------------------------------------------------------
  TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1), 
  since APB1 prescaler is different from 1.   
    TIM2CLK = 2 * PCLK1  
    TIM2CLK = HCLK / 2 = SystemCoreClock /2 

  Note: 
   SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
   Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
   function to update SystemCoreClock variable value. Otherwise, any configuration
   based on this variable will be incorrect.    

  ----------------------------------------------------------- */
	uwPrescalerValue = (uint32_t) ((SystemCoreClock / 1000000) - 1);   //TIM2CLK=1000K
  
  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
  TIM_TimeBaseStructure.TIM_Period = 1000000/TIM2FRE-1;           
  TIM_TimeBaseStructure.TIM_Prescaler = uwPrescalerValue;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


  TIM_OCInitStructure.TIM_OCMode =TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
 // TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse =1000000/TIM2FRE/2-1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
 // TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
 // TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
 // TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;


  TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	// TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  /* TIM2 TRGO selection */
  //TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);
	
  //TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	/*
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	*/
  //NVIC_EnableIRQ(TIM1_IRQn);
  TIM_Cmd(TIM2, ENABLE);

  /* TIM1 Main Output Enable */
 // TIM_CtrlPWMOutputs(TIM2, ENABLE);
}
static void GPIO_Config(void)
{
	GPIO_InitTypeDef       	GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;  //led
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	/*
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;  //AGC_IN
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	*/
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //CTR_LPF
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ; //CTR_G5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 |GPIO_Pin_12 | GPIO_Pin_13 ; //CTR_G4G3G2G1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	

	//GPIO_SetBits(GPIOA,GPIO_Pin_7);
	//GPIO_SetBits(GPIOB,GPIO_Pin_1);
	//GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_12|GPIO_Pin_13);
	GPIO_SetBits(GPIOB,GPIO_Pin_1);
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	
}
/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  uwTimingDelay = nTime;

  while(uwTimingDelay-- != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}
static void USART_Config(void)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	
	//USART_DeInit(USART1);

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
  //RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC , ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD , ENABLE);
	
  
  /* Enable USART clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  
  
  /* USARTx GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART2);
  
  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
 
  /* USARTx configuration ----------------------------------------------------*/
  /* Enable the USART OverSampling by 8 */
 // USART_OverSampling8Cmd(USART1, ENABLE); 
  
  /* USARTx configured as follows:
        - BaudRate = 5250000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 8
		     is: (USART APB Clock / 8) 
			 Example: 
			    - (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
			    - (USART1 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
		   - Maximum BaudRate that can be achieved when using the Oversampling by 16
		     is: (USART APB Clock / 16) 
			 Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
			 Example: (USART1 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud
        - Word Length = 8 Bits
        - one Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
  */ 
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART1, &USART_InitStructure);
         
  /* Enable USART */
  USART_Cmd(USART1, ENABLE);
	
	#if 0
	  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	#endif
}

int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}
void ADC_FifoInit(void)
{
	adcfifo.bytecnt=0;
	adcfifo.readptr=0;
	adcfifo.writeptr=0;
}
void readfifo(uint32_t size)
{
	uint32_t i;
	float32_t val;

	while(adcfifo.bytecnt<FFT_N);

	for(i=0;i<FFT_N;i++)	
	{
		//val = 0.3*adcfifo.buf[(adcfifo.readptr+i-1)%RECEBUFFERSIZE] + 0.2*adcfifo.buf[(adcfifo.readptr+i+1)%RECEBUFFERSIZE] + 0.5*adcfifo.buf[(adcfifo.readptr+i)%RECEBUFFERSIZE];
		fft_source_data[i*2]=(float_t)adcfifo.buf[(adcfifo.readptr+i)%RECEBUFFERSIZE]/4096.0f;
		//fft_source_data[i*2]=val/4096.0f;
		fft_source_data[i*2+1]=0.0f;
	}
}

/*--------------------------------------------------------------------------------------------------------*/
/*  direction :      	FORWARD   add adcfifo.bytecnt  																											*/
/*										BACKWARD  reduce addcfifo.bytecnt																										*/
/*--------------------------------------------------------------------------------------------------------*/
void seek( uint8_t direction,uint32_t lng)
{
	if(direction)
	{
		adcfifo.bytecnt -= lng;
		adcfifo.readptr = (adcfifo.readptr+lng)%RECEBUFFERSIZE;
	}
	else
	{
		adcfifo.bytecnt += lng;
		if(adcfifo.readptr >= lng) adcfifo.readptr -= lng;
		else adcfifo.readptr = adcfifo.readptr+RECEBUFFERSIZE-lng;		
	}
}

void fft(void)
{
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_f32(&arm_cfft_sR_f32_len128, fft_source_data, 0, 1);
	/* Process the data through the Complex Magnitude Module for
	calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fft_source_data, resultOutput, FFT_N);
	/* Calculates maxValue and returns corresponding BIN value */
	arm_max_f32(&resultOutput[1], FFT_N/2-1, &maxValue, &testIndex);
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
