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
static float32_t resultOutput[ FFT_N];
char comp[21]={0x55,0xab,0x57,0x12,0x0c,0xa8,0xd1,0xff,0x00,0xcc,0x89,0x11,0xef,0x88,0x22,0xa};
uint32_t testIndex,testIndex0,testIndex1,value=0,valueback=0,m,error;
float32_t maxValue,maxpower;
float32_t  scale=1;
__IO uint16_t uhADCConvertedValue[4];
/*
const float32_t par[10][6]={
1 , 2 , 1 , 1 , -1.0430928891643865  , 0.87751759754123226 ,       
1 , 2 , 1 , 1 , -0.93052334751353005 , 0.67489777567096199 ,       
1 , 2 , 1 , 1 , -0.84292914159441001 , 0.51723237044751591 ,       
1 , 2 , 1 , 1 , -0.77461529616753233 , 0.39427069725710351 ,       
1 , 2 , 1 , 1 , -0.72152150872393128 , 0.29870440466610387 ,       
1 , 2 , 1 , 1 , -0.68073973620833561 , 0.22529915346331555 ,       
1 , 2 , 1 , 1 , -0.65019112064957252 , 0.17031309815807155 ,       
1 , 2 , 1 , 1 , -0.62840993323306094 , 0.13110799658499458 ,       
1 , 2 , 1 , 1 , -0.61440027570759725 , 0.10589127925059144 ,       
1 , 2 , 1 , 1 , -0.60754362504981518 , 0.09354963412587837 ,  
};
const float32_t Scale[10]={                                              
0.2086061770942115 ,                                          
0.18609360703935804 ,                                         
0.1685758072132765,                                           
0.15491385027239277,                                          
0.14429572398554316 ,                                         
0.13613985431374495,                                          
0.13003049437712474 ,                                         
0.12567451583798339,                                          
0.12287275088574857 ,                                         
0.12150150226901582,   
};
*/
#define ORDER   6
/**********************************************************
const float32_t par[6][6]={

1,  0,  -1,  1,  -0.35202070780445999,  0.69841329456088241,       
1,  0,  -1,  1,   1.6721627431405235,   0.85446220975024145,       
1,  0,  -1,  1,   1.4040495893309388,   0.59609752164307472,       
1,  0,  -1,  1,  -0.13477886598556976,  0.30780879388875387,       
1,  0,  -1,  1,   1.0541296511899381,   0.32330672516072556,       
1,  0,  -1,  1,   0.19620240615733592,  0.091629433857637083  };      
     

const float32_t Scale[6]={ 
                                               
0.5583265104308448,                                           
0.5583265104308448,                                           
0.48067172636464101,                                          
0.48067172636464101,                                          
0.4483392762692337,                                           
0.4483392762692337   };   
************************************************************/
const float32_t par[6][6]={
1,  0,  -1,  1,  -1.4591606304397224,   0.87973649155421341,       
1,  0,  -1,  1,  -0.41846569771945502,  0.82021950735620763,      
1,  0,  -1,  1,  -1.2343280746210685,   0.67568974644958413,       
1,  0,  -1,  1,  -0.49668177233742278,  0.57291704271618549,       
1,  0,  -1,  1,  -0.97174527430205482,  0.52305697268513263,       
1,  0,  -1,  1,  -0.69793946181037536,  0.4726421392880964 };  

const float32_t Scale[6]={ 
0.30719609014376226,                                          
0.30719609014376226,                                          
0.27574557906367186,                                          
0.27574557906367186,                                          
0.26147114722677106,                                          
0.26147114722677106};

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
static void DMAADC_Config(void);
 void DMA_Config(void);
void  CalScale(void);
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
	GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_SET);
	USART_Config();
	TIM3_Config();
	TIM2_Config();
	ADC_Config();   
	CalScale();
	
	//DMAADC_Config();

	/* Start ADC1 Software Conversion */ 
  //ADC_SoftwareStartConv(ADC1);
	
  /* Infinite loop */

  printf("\nSystem clock rate: %d Hz ver=4.2\r\n", SystemCoreClock);
	mm=0;
	//ADC_FifoInit();	
	printf("Search sync ......");
	//GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	for(cnt=0;cnt<0x100;cnt++)
	printf("\nSystem clock rate: %d Hz ver=4.2\r\n", SystemCoreClock);
	
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);

bitsync:
	cnt=0;
	maxpower=0;
	maxpowerindex=0;
	ADC_FifoInit();
	//GPIO_SetBits(GPIOA,GPIO_Pin_1);
	while(1)
	{			
		readfifo(SAMPLE_CNT);
		fft();	
		//if((testIndex >=3 && testIndex<=4))
	//	printf("testIndex=%d ",testIndex);
		if((testIndex >=442 && testIndex<=453))			
		{				
			if(maxValue > maxpower) {maxpower = maxValue; maxpowerindex=adcfifo.readptr;}
		//	GPIO_SetBits(GPIOA,GPIO_Pin_1);
			cnt++;
			
		}	
		//else printf("--%d, ",testIndex);
		//	else GPIO_ResetBits(GPIOA,GPIO_Pin_1);
#if 1			
		//cnt++;
		if(cnt>=8)
		{
			//seek(BACKWARD,adcfifo.readptr-maxpowerindex);
		  break;
		}	
		seek(FORWARD,2);
#endif	
	//	seek(FORWARD,FFT_N);		
	}

	seek(BACKWARD,adcfifo.readptr-maxpowerindex);
	seek(FORWARD,SAMPLE_CNT);
	cnt=0;value=0;
	value =0;m=0;mm=0;error=0;
	while(1)
	{
		readfifo(SAMPLE_CNT); 
		fft();
		//printf("0x%d, ",testIndex);
		#if 1
		if((testIndex >=F1-NEG_OFFSET && testIndex<=F1+POS_OFFSET))
 	  { value |= (0x0<<(4-m));m+=4;}
		else if( testIndex>=F2-NEG_OFFSET && testIndex<=F2+POS_OFFSET)
    { value |= (0x1<<(4-m));m+=4;}
		else if( testIndex>=F3-NEG_OFFSET && testIndex<=F3+POS_OFFSET)
    { value |= (0x2<<(4-m));m+=4;}
		else if( testIndex>=F4-NEG_OFFSET && testIndex<=F4+POS_OFFSET)
    { value |= (0x3<<(4-m));m+=4;}
		else if( testIndex >=F5-NEG_OFFSET && testIndex<=F5+POS_OFFSET)
 	  { value |= (0x4<<(4-m));m+=4;}
		else if( testIndex>=F6-NEG_OFFSET && testIndex<=F6+POS_OFFSET)
    { value |= (0x5<<(4-m));m+=4;}
		else if( testIndex>=F7-NEG_OFFSET && testIndex<=F7+POS_OFFSET)
    { value |= (0x6<<(4-m));m+=4;}
		else if( testIndex>=F8-NEG_OFFSET && testIndex<=F8+POS_OFFSET)
    { value |= (0x7<<(4-m));m+=4;}
		else if( testIndex>=F9-NEG_OFFSET && testIndex<=F9+POS_OFFSET)
    { value |= (0x8<<(4-m));m+=4;}
		else if( testIndex>=F10-NEG_OFFSET && testIndex<=F10+POS_OFFSET)
    { value |= (0x9<<(4-m));m+=4;}	
		else if( testIndex>=F11-NEG_OFFSET && testIndex<=F11+POS_OFFSET)
    { value |= (0xa<<(4-m));m+=4;}
		else if( testIndex >=F12-NEG_OFFSET && testIndex<=F12+POS_OFFSET)
 	  { value |= (0xb<<(4-m));m+=4;}
		else if( testIndex>=F13-NEG_OFFSET && testIndex<=F13+POS_OFFSET)
    { value |= (0xc<<(4-m));m+=4;}
		else if( testIndex>=F14-NEG_OFFSET && testIndex<=F14+POS_OFFSET)
    { value |= (0xd<<(4-m));m+=4;}
		else if( testIndex>=F15-NEG_OFFSET && testIndex<=F15+POS_OFFSET)
    { value |= (0xe<<(4-m));m+=4;}
		else if( testIndex>=F16-NEG_OFFSET && testIndex<=F16+POS_OFFSET)
    { value |= (0xf<<(4-m));m+=4;}
		else 
		{ 
			GPIO_ResetBits(GPIOA,GPIO_Pin_1);
			printf("\r\nsignal lose... m=%d testIndex=%d\r\n",m,testIndex);
			goto bitsync;
		}
		#endif
		
		#if 0
		if((testIndex >=34 && testIndex<=40))
 	  {/*printf("1")*//*GPIO_SetBits(GPIOB,GPIO_Pin_5);*/ value |= (1<<(7-m));m++;}
		else if( testIndex>=50 && testIndex<=56)
    { /*GPIO_ResetBits(GPIOB,GPIO_Pin_5);*//*printf("0");*/m++;}
		else 
		{ 
			GPIO_ResetBits(GPIOA,GPIO_Pin_1);
			printf("\r\nsignal lose... \r\n");
			goto bitsync;
		}
		#endif
		
		if(m>=8) 
		{
			//if(value!=0xaa && value!= 0x7e) 
			{
				printf("0x%02x,",value);
				//printf("%d",value);
				comp[mm]=value;
				if(value - valueback !=1 )
					GPIO_ResetBits(GPIOA,GPIO_Pin_1);
				else
					GPIO_SetBits(GPIOA,GPIO_Pin_1);
				valueback =value;
				
			}
			goto bitsync;
			
		}
		seek(FORWARD,SAMPLE_CNT);
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

static void DMAADC_Config(void)
{
  ADC_InitTypeDef       	ADC_InitStructure;
  ADC_CommonInitTypeDef 	ADC_CommonInitStructure;
  GPIO_InitTypeDef       	GPIO_InitStructure;

 
    
  /* Enable peripheral clocks *************************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	DMA_Config();
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
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC1 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 2;
  ADC_Init(ADC1, &ADC_InitStructure);

  
  /* ADC1 regular channel18 (VBAT) configuration ******************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_15Cycles);

	//ADC_DiscModeChannelCountConfig(ADC1,1);
	//ADC_DiscModeCmd(ADC1,ENABLE);
	//ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);
	//NVIC_EnableIRQ(ADC_IRQn);
	 ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	 ADC_DMACmd(ADC1, ENABLE);


  /* Enable ADC1 **************************************************************/
	//NVIC_EnableIRQ(ADC_IRQn);
  ADC_Cmd(ADC1, ENABLE);
}

 void DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	 __IO uint32_t    timeout = TIMEOUT_MAX;

  DMA_ClearFlag(DMA2_Stream0, DMA_IT_TC);


  DMA_Cmd(DMA2_Stream0, DISABLE);
  DMA_DeInit(DMA2_Stream0);
	
	
  DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADCConvertedValue;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ; //DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	
  DMA_ITConfig(DMA2_Stream0, DMA_IT_TC, ENABLE);

  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);

//  DMA_DoubleBufferModeConfig();
	
	  timeout = TIMEOUT_MAX;
  while ((DMA_GetCmdStatus(DMA2_Stream0) != ENABLE) && (timeout-- > 0))
  {
  }
	  /* Enable the DMA Stream IRQ Channel */
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);     
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
  TIM_TimeBaseStructure.TIM_Period = 195; //390; //195;      //156    
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
	GPIO_SetBits(GPIOA,GPIO_Pin_1);
	
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
	//GPIO_SetBits(GPIOB,GPIO_Pin_13);
	
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

	while(adcfifo.bytecnt<SAMPLE_CNT);

	for(i=0;i<SAMPLE_CNT;i++)	
	{
		//val = 0.3*adcfifo.buf[(adcfifo.readptr+i-1)%RECEBUFFERSIZE] + 0.2*adcfifo.buf[(adcfifo.readptr+i+1)%RECEBUFFERSIZE] + 0.5*adcfifo.buf[(adcfifo.readptr+i)%RECEBUFFERSIZE];
		fft_source_data[i*2]=(float_t)adcfifo.buf[(adcfifo.readptr+i)%RECEBUFFERSIZE]/4096.0f;
		//fft_source_data[i*2]=val/4096.0f;
		fft_source_data[i*2+1]=0.0f;
	}
	
	for(i=SAMPLE_CNT*2;i<FFT_N*2;i++)	
	{
		fft_source_data[i]=0.0f;
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
	arm_cfft_f32(&arm_cfft_sR_f32_len2048, fft_source_data, 0, 1);
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


void  CalScale(void)
{
	uint32_t i;
	
	for(i=0 ; i<ORDER ; i++)
	  scale *= Scale[i];	  
}

float32_t iir(uint32_t u)
{
	static float32_t x[ORDER+2][3]={0,},y[ORDER+2][3]={0,};
	uint32_t num=0;
	float32_t z;
	x[num][0]=(float32_t)u;
	
	
	while(num<ORDER)
	{
		y[num][0] = x[num][0]*par[num][0] +x[num][1]*par[num][1]+x[num][2]*par[num][2] - y[num][1]*par[num][4]-y[num][2]*par[num][5];
		x[num][2]=x[num][1];
		x[num][1]=x[num][0];
		y[num][2]=y[num][1];
		y[num][1]=y[num][0];
		x[num+1][0]=y[num][0];
		num=num+1;

	}
	z=x[num][0]*scale;
	return z;
}
/*
function z = iir(u)
%#codegen
persistent x y;
if isempty(x)
    x=zeros(12,3);
end
if isempty(y)
    y=zeros(12,3);
end

par=[
1  2  1  1  -1.0430928891643865   0.87751759754123226        
1  2  1  1  -0.93052334751353005  0.67489777567096199        
1  2  1  1  -0.84292914159441001  0.51723237044751591        
1  2  1  1  -0.77461529616753233  0.39427069725710351        
1  2  1  1  -0.72152150872393128  0.29870440466610387        
1  2  1  1  -0.68073973620833561  0.22529915346331555        
1  2  1  1  -0.65019112064957252  0.17031309815807155        
1  2  1  1  -0.62840993323306094  0.13110799658499458        
1  2  1  1  -0.61440027570759725  0.10589127925059144        
1  2  1  1  -0.60754362504981518  0.09354963412587837   
];
Scale=[                                                
0.2086061770942115                                           
0.18609360703935804                                          
0.1685758072132765                                           
0.15491385027239277                                          
0.14429572398554316                                          
0.13613985431374495                                          
0.13003049437712474                                          
0.12567451583798339                                          
0.12287275088574857                                          
0.12150150226901582   
];
num=1;
x(num,1)=u;
while num <= 10

y(num,1) = x(num,1)*par(num,1) +x(num,2)*2+x(num,3) - y(num,2)*par(num,5)-y(num,3)*par(num,6);
x(num,3)=x(num,2);
x(num,2)=x(num,1);
y(num,3)=y(num,2);
y(num,2)=y(num,1);
x(num+1,1)=y(num,1);
num=num+1;

%x(2,1)=y(1,1);
%y(2,1) = x(2,1)*par(2,1) +x(2,2)*2+x(2,3) - y(2,2)*par(2,5)-y(2,3)*par(2,6);
%x(2,3)=x(2,2);
%x(2,2)=x(2,1);
%y(2,3)=y(2,2);
%y(2,2)=y(2,1);

%x(3,1)=y(2,1);
%y(3,1) = x(3,1)*par(3,1) +x(3,2)*2+x(3,3) - y(3,2)*par(3,5)-y(3,3)*par(3,6);
%x(3,3)=x(3,2);
%x(3,2)=x(3,1);
%y(3,3)=y(3,2);
%y(3,2)=y(3,1);

%x(4,1)=y(3,1);
%y(4,1) = x(4,1)*par(4,1) +x(4,2)*2+x(4,3) - y(4,2)*par(4,5)-y(4,3)*par(4,6);
%x(4,3)=x(4,2);
%x(4,2)=x(4,1);
%y(4,3)=y(4,2);
%y(4,2)=y(4,1);

%x(5,1)=y(4,1);
%y(5,1) = x(5,1)*par(5,1) +x(5,2)*2+x(5,3) - y(5,2)*par(5,5)-y(5,3)*par(5,6);
%x(5,3)=x(5,2);
%x(5,2)=x(5,1);
%y(5,3)=y(5,2);
%y(5,2)=y(5,1);
end

%z = y(5,1)*0.19663807054877341*0.16131994340289288*0.13993928686685159*0.12764772437302829*0.12201343969355956;
z=x(num,1)*Scale(1)*Scale(2)*Scale(3)*Scale(4)*Scale(5)*Scale(6)*Scale(7)*Scale(8)*Scale(9)*Scale(10);

*/  
/**
  * @}
  */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
