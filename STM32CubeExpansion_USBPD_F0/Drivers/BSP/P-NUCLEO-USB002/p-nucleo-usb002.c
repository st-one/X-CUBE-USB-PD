/**
  ******************************************************************************
  * @file    p-nucleo-usb002.c
  * @author  System Lab
  * @brief   This file provides set of functions to manage peripherals on
  *          P_NUCLEO_USB002 board.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
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
#if defined(USBPD_STUSB1605)
#include "usbpd_prl_hw.h"
#endif /* USBPD_STUSB1605 */
#include "p-nucleo-usb002.h"
  

/**
 * @addtogroup P_NUCLEO_USB002
 * @{
 * */

/* Private typedef -----------------------------------------------------------*/
#if defined(USBPD_STUSB1605)
#else
extern SPI_HandleTypeDef hspi2;
#endif /* USBPD_STUSB1605 */
#ifdef HAL_UART_MODULE_ENABLED
extern UART_HandleTypeDef huart_handle;
#endif /* HAL_UART_MODULE_ENABLED */

/* Private define ------------------------------------------------------------*/

#define BAUDRATE	115200		/* BaudRate for the UART */

/**
 * @brief Usart used by P_NUCLEO_USB002
 * */
#define USBPD_BSP_USART			  USART1

/**
 * @brief Clock Enable Macro
 * */
#define USBPD_BSP_USARTCLK_ENABLE	        __HAL_RCC_USART1_CLK_ENABLE

/**
 * @brief PIN RELATED MACROS
 * @{
 * */
#define USART_TX_PORT			        GPIOA
#define USART_TX_PIN			        GPIO_PIN_9
#define USART_RX_PORT			        GPIOA
#define USART_RX_PIN			        GPIO_PIN_10
#define USART_PIN_GPIOAF		        GPIO_AF1_USART1
#define USART_IRQ			        USART1_IRQn

/**
 * @}
 * */
 
 /* 
  * NOTE: BAUDRATE, P_NUCLEO_USB002_USART have been defined but they haven't used yet.
  *       usart.c and usart.h files have to be modified according to these definitions
  */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#if defined(USBPD_STUSB1605)
TCPC_DrvTypeDef* DevicesDrivers[USBPD_PORT_COUNT] = {
  &stusb1605_tcpc_drv,
#if USBPD_PORT_COUNT == 2
  &fusb305_tcpc_drv,
#endif /*USBPD_PORT_COUNT == 2*/
};
#endif /* USBPD_STUSB1605 */

/**
 * @brief Vector storing informations on pins controlling leds of P_NUCLEO_USB002
 * */

USBPD_BSP_GPIOPins_TypeDef USBPD_BSP_LEDs[USBPD_BSP_LEDn] =
    {
      USBPD_BSP_PIN(GPIOA,5),           /* LED1: generic */
      
      USBPD_BSP_PIN(GPIOC,5),           /* LED01: PORT0_ROLE */
      USBPD_BSP_PIN(GPIOB,1),           /* LED02: PORT0_VBUS */
      USBPD_BSP_PIN(GPIOB,2),           /* LED03: PORT0_CCx  */
      
      USBPD_BSP_PIN(GPIOC,14),          /* LED11: PORT1_ROLE */
      USBPD_BSP_PIN(GPIOA,6),           /* LED12: PORT1_VBUS */
      USBPD_BSP_PIN(GPIOC,15)           /* LED13: PORT1_CCx  */
    } ;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* ************************************************************************* */
/* Led Procedures and functions                                              */

/**
  * @brief  Configures P_NUCLEO_USB002 LED GPIO.
  * @param  None
  * @retval None
  */

  void USBPD_BSP_LED_Init(void)
  {
    GPIO_InitTypeDef  GPIO_InitStruct;
    uint8_t led=0;

    /* Common values for Leds GPIO */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;    
    
    for(led=0;led<USBPD_BSP_LEDn;led++)
    {
        /* Configure the GPIO pin */
        GPIO_InitStruct.Pin = USBPD_BSP_LEDs[led].GPIO_Pin;

        /* Init the associated GPIO */
        HAL_GPIO_Init(USBPD_BSP_LEDs[led].GPIOx, &GPIO_InitStruct);
        /* Turn the led off */
        USBPD_BSP_LED_Off((USBPD_BSP_Led_TypeDef)led);
    }
  }


/**
  * @brief  Turns selected LED On or Off.
  * @param  Led: Specifies the Led to be set on.
  * @param  Value: value to set the led on or off.
  * @retval None
  */
  void USBPD_BSP_LED_Set(USBPD_BSP_Led_TypeDef Led, uint8_t Value)
  {
    HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, Value ? GPIO_PIN_RESET : GPIO_PIN_SET);
  }

  
/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on.
  * @retval None
  */  
  
  void USBPD_BSP_LED_On(USBPD_BSP_Led_TypeDef Led)
  {
    HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, GPIO_PIN_RESET);
  }

  
/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off.
  * @retval None
  */  
  
  void USBPD_BSP_LED_Off(USBPD_BSP_Led_TypeDef Led)
  {
    HAL_GPIO_WritePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin, GPIO_PIN_SET);
  }

  
/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled.
  * @retval None
  */ 
  
  void USBPD_BSP_LED_Toggle(USBPD_BSP_Led_TypeDef Led)
  {
    HAL_GPIO_TogglePin(USBPD_BSP_LEDs[Led].GPIOx, USBPD_BSP_LEDs[Led].GPIO_Pin);
  }

  

#if defined(HAL_UART_MODULE_ENABLED)
/**
  * @brief  Configures the UART used by the P_NUCLEO_USB002
  * @retval None
  */

void USBPD_BSP_UART_Init(void)
{
    /* Init huart_usbpdm1 */
    huart_handle.Instance = USBPD_BSP_USART;
    huart_handle.Init.BaudRate = BAUDRATE;
    huart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    huart_handle.Init.StopBits = UART_STOPBITS_1;
    huart_handle.Init.Parity = UART_PARITY_NONE;
    huart_handle.Init.Mode = UART_MODE_TX_RX;
    huart_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    huart_handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart_handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    /* Init of the peripheral */
    HAL_UART_Init(&huart_handle);
}

/*
 * @brief Init the low level hardware : GPIO, CLOCK
 * */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* Peripheral clock enable */
    USBPD_BSP_USARTCLK_ENABLE();

    /* USART GPIO Configuration */
    GPIO_InitStruct.Pin = USART_TX_PIN | USART_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = USART_PIN_GPIOAF;
    HAL_GPIO_Init(USART_TX_PORT, &GPIO_InitStruct);
    
    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(USART_IRQ, 3, 0);
    HAL_NVIC_EnableIRQ(USART_IRQ);
}
#endif /* HAL_UART_MODULE_ENABLED */

#if defined(USBPD_STUSB1605)
void USBPD_HW_IF_GPIO_Set(USBPD_BSP_GPIOPins_TypeDef gpio, GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(gpio.GPIOx, gpio.GPIO_Pin, PinState);
}
void USBPD_HW_IF_GPIO_On(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Sets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_SET);
}

void USBPD_HW_IF_GPIO_Off(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Resets the pin */
  USBPD_HW_IF_GPIO_Set(gpio, GPIO_PIN_RESET);
}

void USBPD_HW_IF_GPIO_Toggle(USBPD_BSP_GPIOPins_TypeDef gpio)
{
  /* Toggle the pin */
  HAL_GPIO_TogglePin(gpio.GPIOx, gpio.GPIO_Pin);
}

void BSP_USBPD_SetVoltage(uint32_t Port, uint32_t Voltage)
{
#warning "[YMA] TBD"
}
/**
  * @brief  Get the TCPC Drivers
  * @param  PortNum     Index of current used port
  * @param  TCPC_Driver Pointer on the TCPC drivers
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_TCPCI_GetDevicesDrivers(uint8_t PortNum, TCPC_DrvTypeDef **TCPC_Driver)
{
  *TCPC_Driver = DevicesDrivers[PortNum];

  return USBPD_OK;
}
#endif /* USBPD_STUSB1605 */
/*
 *  }@ 
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
