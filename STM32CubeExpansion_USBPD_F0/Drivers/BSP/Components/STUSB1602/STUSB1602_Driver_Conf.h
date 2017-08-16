/**
  ******************************************************************************
  * @file    STUSB1602_Driver_Conf.h
  * @author  System Lab
  * @version V1.2.1
  * @date    24-Apr-2017
  * @brief   This file contains the headers of usbpd_hw_if.h.
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

#ifndef __STUSB1602_DRIVER_CONF_H_
#define __STUSB1602_DRIVER_CONF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usbpd_conf.h"

/* Exported typedef ----------------------------------------------------------*/

/* Exported define -----------------------------------------------------------*/
//#define STUSB16xx_I2CxHandle    hi2c_driver
//#define STUSB1602_I2C_Addn      2
//#define STUSB1602_I2C_Add_0     0x28
//#define STUSB1602_I2C_Add_1     0x29
//#define TIMEOUT_MAX             2000 /*<! The value of the maximal timeout for BUS waiting loops */
   
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define STUSB1602_I2C_Add(__PORT__)		((__PORT__ == 0) ? STUSB1602_I2C_Add_0 : STUSB1602_I2C_Add_1 )

   
/* Exported variables --------------------------------------------------------*/
//static I2C_HandleTypeDef STUSB16xx_I2CxHandle;



/* to be filled
#define WriteReg(...) HAL_I2C_Mem_Write(...)
*/

/* Exported functions --------------------------------------------------------*/
//void STUSB1602_Driver_Init(I2C_HandleTypeDef I2CxHandle);



#ifdef __cplusplus
}
#endif

#endif /* __STUSB1602_DRIVER_CONF_H_ */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
