/**
  ******************************************************************************
  * @file    stm32f0xx_hal_msp_template.c
  * @author  System Lab
  * @version V1.1.0
  * @date    22-June-2016
  * @brief   This file contains HW interface MSP functions template file.
  *          This file should be copied to the application folder and modified 
  *          as follows:
  *            - Rename it to 'stm32f0xx_hal_msp.c'.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usbpd_hw_if.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
* @addtogroup PROVIDER-CLI
* @{
* */

/**
* Initializes the Global MSP.
*/
void HAL_MspInit(void)
{

}

/**
* @brief ADC MSP Initialization
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{

}

/**
* @brief ADC MSP De-initialization
* @param hadc: ADC handle pointer
* @retval None
*/
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{

}

/**
* @brief COMP MSP Initialization
* @param hcomp: COMP handle pointer
* @retval None
*/
void HAL_COMP_MspInit(COMP_HandleTypeDef* hcomp)
{

}

/**
* @brief COMP MSP De-initialization
* @param hcomp: COMP handle pointer
* @retval None
*/
void HAL_COMP_MspDeInit(COMP_HandleTypeDef* hcomp)
{

}

/**
* @brief SPI MSP Initialization
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

}

/**
* @brief SPI MSP De-initialization
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

}


/**
* @brief CRC MSP Initialization
* @param hcrc: CRC handle pointer
* @retval None
*/
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc)
{

}

/**
* @brief CRC MSP De-initialization
* @param hcrc: CRC handle pointer
* @retval None
*/
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc)
{

}

/**
* @brief TIM MSP Initialization
* @param htim_base: TIM handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

}

/**
* @brief TIM MSP De-initialization
* @param htim_base: TIM handle pointer
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

}

/** @} */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
