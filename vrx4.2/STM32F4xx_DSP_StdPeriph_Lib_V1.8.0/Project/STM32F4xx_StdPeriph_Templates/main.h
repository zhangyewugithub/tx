/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/main.h 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Header for main.c module
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include <stdio.h>
#include "arm_math.h"
#include <math.h>  
#include "arm_const_structs.h"


/* Exported types ------------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define 	TRUE						1
#define		FAIL						0
#define 	FORWARD						1
#define 	BACKWARD					0 
#define   	BAUD_RATE					20                                              
#define 	BIT1_FREQ 					(BAUD_RATE)
#define 	BIT0_FREQ 					(BAUD_RATE)  
#define  	SAMPLE_CNT				80
#define  	FFT_N  						2048
#define 	FRAME_SYNC_CODE 			0x7e
#define   	ADC_SAMPLES_FREQ			(BUAD_RATE*FFT_N/4)
#define   	ONE_BIT_TIME				(1/BAUD_RATE)     
#define 	RECEBUFFERSIZE				(FFT_N*2)
#define		THRESHOLD					(15253*4/5)
#define		DC_COMPONENT        		(0)
#define   	SCALE						(3/4)
#define   TIM2FRE						 6400
#define    DUTY							(1/2)


#define  FRE_RESOLUTION   (0.25)
#define F1                               (64/FRE_RESOLUTION)
#define F2                               (66/FRE_RESOLUTION)
#define F3                               (68/FRE_RESOLUTION)
#define F4                               (70/FRE_RESOLUTION)
#define F5                               (72/FRE_RESOLUTION)
#define F6                               (74/FRE_RESOLUTION)
#define F7                               (76/FRE_RESOLUTION)
#define F8                               (78/FRE_RESOLUTION)
#define F9                               (80/FRE_RESOLUTION)
#define F10                              (82/FRE_RESOLUTION)
#define F11                              (84/FRE_RESOLUTION)
#define F12                              (86/FRE_RESOLUTION)
#define F13                              (88/FRE_RESOLUTION)
#define F14                              (90/FRE_RESOLUTION)
#define F15                              (92/FRE_RESOLUTION)
#define F16                              (94/FRE_RESOLUTION)
#define F17                              (96/FRE_RESOLUTION)

#define POS_OFFSET                       (-3)
#define NEG_OFFSET											 (10)

#define TIMEOUT_MAX              ((uint32_t)10000) 
typedef struct
{
	uint32_t readptr;
	uint32_t writeptr;
	uint32_t bytecnt;
	//q31_t buf[RECEBUFFERSIZE];
	float32_t	buf[RECEBUFFERSIZE];
}adc_buffer_t;

#define ADC1_DR_ADDRESS          ((uint32_t)0x4001224C)

/* Exported functions ------------------------------------------------------- */
void TimingDelay_Decrement(void);
float32_t iir(uint32_t u);


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
