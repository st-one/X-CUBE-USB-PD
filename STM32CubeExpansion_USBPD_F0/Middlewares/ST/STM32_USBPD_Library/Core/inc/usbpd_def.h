/**
  ******************************************************************************
  * @file    usbpd_def.h
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Global defines for USB-PD libarary
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#ifndef __USBPD_DEF_H_
#define __USBPD_DEF_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#if defined(STM32F072xB) || defined(STM32F051x8)
#include "stm32f0xx.h"
#elif defined(STM32F334x8)
#include "stm32f3xx.h"
#else
#error "Add include for the serie"
#endif

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_CORE
  * @{
  */

/** @addtogroup USBPD_CORE_DEF
  * @{
  */

/* Internal macros --------------------------------------------------------*/
/** @defgroup USBPD_CORE_DEF_Internal_Macros USBPD Core DEF Internal Macros
  * @{
  */
#define DIV_ROUND_UP(x, y) (((x) + ((y) - 1)) / (y))
#define MV2ADC(__X__)           ( (__X__*4095) / 3300 )
#define ADC2MV(__X__)           ( (__X__*3300) / 4095 )
 
 /* Macros for integer division with various rounding variants default integer 
    division rounds down. */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define OTHER_CC(__CC__)                ( ((CCxPin_TypeDef)(__CC__))==CC1 ? CC2 : ( (((CCxPin_TypeDef)(__CC__))==CC2) ? CC1 : CCNONE ) ) 

#define USPBPD_WRITE32(addr,data)   do {                                                                       \
                                         uint8_t bindex;                                                       \
                                         for(bindex = 0; bindex < 4; bindex++)                                 \
                                         {                                                                     \
                                           ((uint8_t *)addr)[bindex] = ((data >> (8 * bindex)) & 0x000000FF);  \
                                         }                                                                     \
                                       } while(0);

#define LE16(addr) (((uint16_t)(*((uint8_t *)(addr))))\
                                             + (((uint16_t)(*(((uint8_t *)(addr)) + 1))) << 8))

#define LE32(addr) ((((uint32_t)(*(((uint8_t *)(addr)) + 0))) + \
                                              (((uint32_t)(*(((uint8_t *)(addr)) + 1))) << 8) + \
                                              (((uint32_t)(*(((uint8_t *)(addr)) + 2))) << 16) + \
                                              (((uint32_t)(*(((uint8_t *)(addr)) + 3))) << 24)))

/* USB PD DEBUG macros */
#if (USBPD_DEBUG_LEVEL > 0)
#define USBPD_UsrLog(...)   printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBPD_UsrLog(...)
#endif 


#if (USBPD_DEBUG_LEVEL > 1)
#define USBPD_ErrLog(...)   printf("ERROR: ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBPD_ErrLog(...)
#endif 


#if (USBPD_DEBUG_LEVEL > 2)
#define USBPD_DbgLog(...)   printf("DEBUG : ") ;\
                            printf(__VA_ARGS__);\
                            printf("\n");
#else
#define USBPD_DbgLog(...)
#endif

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup USBPD_CORE_DEF_Exported_Constants USBPD Core DEF Exported Constants
  * @{
  */

#define USBPD_PORT_0                     (0U)              /*!< Port 0 identifier */
#define USBPD_PORT_1                     (1U)              /*!< Port 1 identifier */

#define USBPD_MAX_RX_BUFFER_SIZE         ((uint32_t)30)    /*!< Maximum size of Rx buffer */
#define USBPD_MAX_TX_BUFFER_SIZE         ((uint32_t)30)    /*!< Maximum size of Tx buffer */

#define USBPD_MAX_NB_PDO                 (7U)              /*!< Maximum number of supported Power Data Objects: fix by the Specification */
#define BIST_CARRIER_MODE_MS             (50U)             /*!< Time in ms of the BIST signal*/

/*
 * Maximum size of a Power Delivery packet (in bits on the wire) :
 *    16-bit header + 0..7 32-bit data objects  (+ 4b5b encoding)
 *    64-bit preamble + SOP (4x 5b) + header (16-bit) + message in 4b5b + 32-bit CRC  + EOP (1x 5b)
 * =  64bit           + 4*5bit      + 16bit * 5/4 + 7 * 32bit * 5/4 + 32bit * 5/4 + 5
 */
#define PHY_BIT_LEN            ((uint16_t)429)
#define PHY_MAX_RAW_SIZE       ((uint16_t)((PHY_BIT_LEN*2) + 3))


/** @defgroup USBPD_PDO_Index_And_Mask_Constants Index and Mask constants used in PDO bits handling
  * @{
  */
#define USBPD_PDO_TYPE_Pos                           (30U)                                                     /*!< PDO Type bits position                          */
#define USBPD_PDO_TYPE_Msk                           (0x3U << USBPD_PDO_TYPE_Pos)                              /*!< PDO Type bits mask : 0xC0000000                 */
#define USBPD_PDO_TYPE_FIXED                         (USBPD_CORE_PDO_TYPE_FIXED << USBPD_PDO_TYPE_Pos)         /*!< PDO Type = FIXED                                */
#define USBPD_PDO_TYPE_BATTERY                       (USBPD_CORE_PDO_TYPE_BATTERY << USBPD_PDO_TYPE_Pos)       /*!< PDO Type = BATTERY                              */
#define USBPD_PDO_TYPE_VARIABLE                      (USBPD_CORE_PDO_TYPE_VARIABLE << USBPD_PDO_TYPE_Pos)      /*!< PDO Type = VARIABLE                             */

/* Source Fixed type PDO elments */
#define USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Pos          (29U)                                                     /*!< DRP Support bit position                        */
#define USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Msk          (0x1U << USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Pos)             /*!< DRP Support bit mask : 0x20000000               */
#define USBPD_PDO_SRC_FIXED_DRP_NOT_SUPPORTED        (0U)                                                      /*!< DRP not supported                               */
#define USBPD_PDO_SRC_FIXED_DRP_SUPPORTED            USBPD_PDO_SRC_FIXED_DRP_SUPPORT_Msk                       /*!< DRP supported                                   */

#define USBPD_PDO_SRC_FIXED_USBSUSPEND_Pos           (28U)                                                     /*!< USB Suspend Support bit position                */
#define USBPD_PDO_SRC_FIXED_USBSUSPEND_Msk           (0x1U << USBPD_PDO_SRC_FIXED_USBSUSPEND_Pos)              /*!< USB Suspend Support bit mask : 0x10000000       */
#define USBPD_PDO_SRC_FIXED_USBSUSPEND_NOT_SUPPORTED (0U)                                                      /*!< USB Suspend not supported                       */
#define USBPD_PDO_SRC_FIXED_USBSUSPEND_SUPPORTED     USBPD_PDO_SRC_FIXED_USBSUSPEND_Msk                        /*!< USB Suspend supported                           */

#define USBPD_PDO_SRC_FIXED_EXT_POWER_Pos            (27U)                                                     /*!< External Power available bit position           */
#define USBPD_PDO_SRC_FIXED_EXT_POWER_Msk            (0x1U << USBPD_PDO_SRC_FIXED_EXT_POWER_Pos)               /*!< External Power available bit mask : 0x08000000  */
#define USBPD_PDO_SRC_FIXED_EXT_POWER_NOT_AVAILABLE  (0U)                                                      /*!< External Power not available                    */
#define USBPD_PDO_SRC_FIXED_EXT_POWER_AVAILABLE      USBPD_PDO_SRC_FIXED_EXT_POWER_Msk                         /*!< External Power available                        */

#define USBPD_PDO_SRC_FIXED_USBCOMM_Pos              (26U)                                                     /*!< USB Communication Support bit position          */
#define USBPD_PDO_SRC_FIXED_USBCOMM_Msk              (0x1U << USBPD_PDO_SRC_FIXED_USBCOMM_Pos)                 /*!< USB Communication Support bit mask : 0x04000000 */
#define USBPD_PDO_SRC_FIXED_USBCOMM_NOT_SUPPORTED    (0U)                                                      /*!< USB Communication not supported                 */
#define USBPD_PDO_SRC_FIXED_USBCOMM_SUPPORTED        USBPD_PDO_SRC_FIXED_USBCOMM_Msk                           /*!< USB Communication supported                     */

#define USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Pos          (25U)                                                     /*!< Dual Role Data Support bit position             */
#define USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Msk          (0x1U << USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Pos)             /*!< Dual Role Data Support bit mask : 0x02000000    */
#define USBPD_PDO_SRC_FIXED_DRD_NOT_SUPPORTED        (0U)                                                      /*!< Dual Role Data not supported                    */
#define USBPD_PDO_SRC_FIXED_DRD_SUPPORTED            USBPD_PDO_SRC_FIXED_DRD_SUPPORT_Msk                       /*!< Dual Role Data supported                        */

#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos          (20U)                                                             /*!< Peak Current info bits position            */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_Msk          (0x3U << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos)                     /*!< Peak Current info bits mask : 0x00300000   */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_EQUAL        (USBPD_CORE_PDO_PEAKEQUAL << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Equal to Ioc           */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_OVER1        (USBPD_CORE_PDO_PEAKOVER1 << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Overload Cap 1         */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_OVER2        (USBPD_CORE_PDO_PEAKOVER2 << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Overload Cap 2         */
#define USBPD_PDO_SRC_FIXED_PEAKCURRENT_OVER3        (USBPD_CORE_PDO_PEAKOVER3 << USBPD_PDO_SRC_FIXED_PEAKCURRENT_Pos) /*!< Peak Current info : Overload Cap 3         */

#define USBPD_PDO_SRC_FIXED_VOLTAGE_Pos              (10U)                                                     /*!< Voltage in 50 mV units bits position               */
#define USBPD_PDO_SRC_FIXED_VOLTAGE_Msk              (0x3FFU << USBPD_PDO_SRC_FIXED_VOLTAGE_Pos)               /*!< Voltage in 50 mV units bits mask : 0x000FFC00      */

#define USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos          (0U)                                                      /*!< Max current in 10 mA units bits position           */
#define USBPD_PDO_SRC_FIXED_MAX_CURRENT_Msk          (0x3FFU << USBPD_PDO_SRC_FIXED_MAX_CURRENT_Pos)           /*!< Max current in 10 mA units bits mask : 0x000003FF  */

/* Source Variable type PDO elments */
#define USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos       (20U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SRC_VARIABLE_MAX_VOLTAGE_Pos)        /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos       (10U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SRC_VARIABLE_MIN_VOLTAGE_Pos)        /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos       (0U)                                                      /*!< Max current in 10 mA units bits position           */
#define USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Msk       (0x3FFU << USBPD_PDO_SRC_VARIABLE_MAX_CURRENT_Pos)        /*!< Max current in 10 mA units bits mask : 0x000003FF  */

/* Source Battery type PDO elments */
#define USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Pos        (20U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SRC_BATTERY_MAX_VOLTAGE_Pos)         /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Pos        (10U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SRC_BATTERY_MIN_VOLTAGE_Pos)         /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SRC_BATTERY_MAX_POWER_Pos          (0U)                                                      /*!< Max allowable power in 250mW units bits position          */
#define USBPD_PDO_SRC_BATTERY_MAX_POWER_Msk          (0x3FFU << USBPD_PDO_SRC_BATTERY_MAX_POWER_Pos)           /*!< Max allowable power in 250mW units bits mask : 0x000003FF */

/* Sink Fixed type PDO elments */
#define USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Pos          (29U)                                                     /*!< DRP Support bit position                        */
#define USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Msk          (0x1U << USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Pos)             /*!< DRP Support bit mask : 0x20000000               */
#define USBPD_PDO_SNK_FIXED_DRP_NOT_SUPPORTED        (0U)                                                      /*!< DRP not supported                               */
#define USBPD_PDO_SNK_FIXED_DRP_SUPPORTED            USBPD_PDO_SNK_FIXED_DRP_SUPPORT_Msk                       /*!< DRP supported                                   */

#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Pos           (28U)                                                    /*!< Higher capability support bit position          */
#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Msk           (0x1U << USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Pos)            /*!< Higher capability support bit mask : 0x10000000 */
#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_NOT_SUPPORTED (0U)                                                     /*!< Higher capability not supported                 */
#define USBPD_PDO_SNK_FIXED_HIGHERCAPAB_SUPPORTED     USBPD_PDO_SNK_FIXED_HIGHERCAPAB_Msk                      /*!< Higher capability supported                     */

#define USBPD_PDO_SNK_FIXED_EXT_POWER_Pos            (27U)                                                     /*!< External Power available bit position           */
#define USBPD_PDO_SNK_FIXED_EXT_POWER_Msk            (0x1U << USBPD_PDO_SNK_FIXED_EXT_POWER_Pos)               /*!< External Power available bit mask : 0x08000000  */
#define USBPD_PDO_SNK_FIXED_EXT_POWER_NOT_AVAILABLE  (0U)                                                      /*!< External Power not available                    */
#define USBPD_PDO_SNK_FIXED_EXT_POWER_AVAILABLE      USBPD_PDO_SNK_FIXED_EXT_POWER_Msk                         /*!< External Power available                        */

#define USBPD_PDO_SNK_FIXED_USBCOMM_Pos              (26U)                                                     /*!< USB Communication Support bit position          */
#define USBPD_PDO_SNK_FIXED_USBCOMM_Msk              (0x1U << USBPD_PDO_SNK_FIXED_USBCOMM_Pos)                 /*!< USB Communication Support bit mask : 0x04000000 */
#define USBPD_PDO_SNK_FIXED_USBCOMM_NOT_SUPPORTED    (0U)                                                      /*!< USB Communication not supported                 */
#define USBPD_PDO_SNK_FIXED_USBCOMM_SUPPORTED        USBPD_PDO_SNK_FIXED_USBCOMM_Msk                           /*!< USB Communication supported                     */

#define USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Pos          (25U)                                                     /*!< Dual Role Data Support bit position             */
#define USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Msk          (0x1U << USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Pos)             /*!< Dual Role Data Support bit mask : 0x02000000    */
#define USBPD_PDO_SNK_FIXED_DRD_NOT_SUPPORTED        (0U)                                                      /*!< Dual Role Data not supported                    */
#define USBPD_PDO_SNK_FIXED_DRD_SUPPORTED            USBPD_PDO_SNK_FIXED_DRD_SUPPORT_Msk                       /*!< Dual Role Data supported                        */

#define USBPD_PDO_SNK_FIXED_VOLTAGE_Pos              (10U)                                                     /*!< Voltage in 50 mV units bits position               */
#define USBPD_PDO_SNK_FIXED_VOLTAGE_Msk              (0x3FFU << USBPD_PDO_SNK_FIXED_VOLTAGE_Pos)               /*!< Voltage in 50 mV units bits mask : 0x000FFC00      */

#define USBPD_PDO_SNK_FIXED_OP_CURRENT_Pos           (0U)                                                      /*!< Operational current in 10 mA units bits position           */
#define USBPD_PDO_SNK_FIXED_OP_CURRENT_Msk           (0x3FFU << USBPD_PDO_SNK_FIXED_OP_CURRENT_Pos)            /*!< Operational current in 10 mA units bits mask : 0x000003FF  */

/* Sink Variable type PDO elments */
#define USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Pos       (20U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SNK_VARIABLE_MAX_VOLTAGE_Pos)        /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Pos       (10U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Msk       (0x3FFU << USBPD_PDO_SNK_VARIABLE_MIN_VOLTAGE_Pos)        /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Pos        (0U)                                                      /*!< Operational current in 10 mA units bits position           */
#define USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Msk        (0x3FFU << USBPD_PDO_SNK_VARIABLE_OP_CURRENT_Pos)         /*!< Operational current in 10 mA units bits mask : 0x000003FF  */

/* Sink Battery type PDO elments */
#define USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Pos        (20U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SNK_BATTERY_MAX_VOLTAGE_Pos)         /*!< Max Voltage in 50 mV units bits mask : 0x3FF00000  */

#define USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Pos        (10U)                                                     /*!< Max Voltage in 50 mV units bits position           */
#define USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Msk        (0x3FFU << USBPD_PDO_SNK_BATTERY_MIN_VOLTAGE_Pos)         /*!< Max Voltage in 50 mV units bits mask : 0x000FFC00  */

#define USBPD_PDO_SNK_BATTERY_OP_POWER_Pos           (0U)                                                      /*!< Operational power in 250mW units bits position          */
#define USBPD_PDO_SNK_BATTERY_OP_POWER_Msk           (0x3FFU << USBPD_PDO_SNK_BATTERY_OP_POWER_Pos)            /*!< Operational power in 250mW units bits mask : 0x000003FF */

#define USBPD_EXTENDED_MESSAGE                       (0x80U)                                                   /*!< Flag to indicate that it is a extended message     */

/**
  * @}
  */

/**
  * @}
  */

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_CORE_DEF_Exported_TypeDef USBPD Core DEF Exported TypeDef
  * @{
  */
/**
  * @brief USB Power Delivery Status structures definition
  */
typedef enum
{
  USBPD_OK         = 0x00,
  USBPD_ERROR      = 0x01,
  USBPD_BUSY       = 0x02,
  USBPD_TIMEOUT    = 0x03,
  USBPD_ACK        = 0x04,
  USBPD_NAK        = 0x05,
  USBPD_GOODCRC    = 0x06,
  USBPD_FAIL       = 0x07
}USBPD_StatusTypeDef;

/**
  * @brief USB PD Rx Phy Status structures definition
  */
typedef enum 
{
  RX_OK,
  RX_ERROR,
  RX_ERROR_WRONG_ORDERSET,
  RX_ERROR_WRONG_SYMBOL,
  RX_ERROR_MISSING_EOP,
  RX_ERROR_CRC_FAILED
}USBPD_PHY_RX_StatusTypeDef;

/**
  * @brief USB PD CC lines structures definition
  */
typedef enum
{
  CCNONE      = 0,
  CC1         = 1,
  CC2         = 2
}CCxPin_TypeDef;


/** @defgroup USBPD_SpecRev_TypeDef USB PD Specification Revision structure definition
  * @brief  USB PD Specification Revision structure definition
  * @{
  */
typedef enum
{
  USBPD_SPECIFICATION_REV1               = 0x00,  /*!< Revision 1.0      */
  USBPD_SPECIFICATION_REV2               = 0x01,  /*!< Revision 2.0      */
  //USBPD_SPECIFICATION_REV3               = 0x02   /*!< Revision 3.0      */  
}USBPD_SpecRev_TypeDef;
/** 
  * @}
  */


/**
  * @brief USB PD CC lines HW condition
  */
typedef enum
{
  HW_Detachment                         = 0x00,    /*!< Nothing attached   */
  HW_Attachment                         = 0x01,    /*!< Sink attached   */    
  HW_PwrCable_NoSink_Attachment         = 0x02,    /*!< Powered cable without Sink attached   */
  HW_PwrCable_Sink_Attachment           = 0x03,    /*!< Powered cable with Sink or VCONN-powered Accessory attached   */
  HW_Debug_Attachment                   = 0x04,    /*!< Debug Accessory Mode attached   */
  HW_AudioAdapter_Attachment            = 0x05     /*!< Audio Adapter Accessory Mode attached   */ 
}CAD_HW_Condition_TypeDef;


/** @defgroup USBPD_PortDataRole_TypeDef USB PD Port Data Role Types structure definition
  * @brief  USB PD Port Data Role Types structure definition
  * @{
  */
typedef enum
{
  USBPD_PORTDATAROLE_UFP                 = 0x00,  /*!< UFP        */
  USBPD_PORTDATAROLE_SOP1_SOP2           = USBPD_PORTDATAROLE_UFP,  /*!<  For all other SOP* Packets the Port Data Role
                                                                          field is Reserved and shall be set to zero.  */
  USBPD_PORTDATAROLE_DFP                 = 0x01   /*!< DFP        */
}USBPD_PortDataRole_TypeDef;
/** 
  * @}
  */

/**
  * @brief Sink CC pins Multiple Source Current Advertisements
  */
typedef enum
{
  vRd_Undefined     = 0x00,    /*!< Port Power Role Source   */
  vRd_USB           = 0x01,    /*!< Default USB Power   */    
  vRd_1_5A          = 0x02,    /*!< USB Type-C Current @ 1.5 A   */
  vRd_3_0A          = 0x03,    /*!< USB Type-C Current @ 3 A   */
}CAD_SNK_Source_Current_Adv_Typedef;


/**
  * @brief USB PD SOP Message Types Structure definition
  */
typedef enum
{
  USBPD_SOPTYPE_SOP            = 0,     /**< SOP*  MESSAGES               */
  USBPD_SOPTYPE_SOP1           = 1,     /**< SOP'  MESSAGES               */
  USBPD_SOPTYPE_SOP2           = 2,     /**< SOP'' MESSAGES               */
  USBPD_SOPTYPE_SOP1_DEBUG     = 3,     /**< SOP'  DEBUG_MESSAGES         */
  USBPD_SOPTYPE_SOP2_DEBUG     = 4,     /**< SOP'' DEBUG_MESSAGES         */
  USBPD_SOPTYPE_HARD_RESET     = 5,     /**< HARD RESET MESSAGE           */
  USBPD_SOPTYPE_CABLE_RESET    = 6,     /**< CABLE RESET MESSAGE          */
  USBPD_SOPTYPE_BIST_MODE_2    = 7,     /**< BIST_MODE2 MESSAGE           */
  USBPD_SOPTYPE_INVALID        = 0xFF,  /**< Invalid type                 */
} USBPD_SOPType_TypeDef;

/**
  * @brief USB PD Port Power Role Types structure definition
  *
  */
typedef enum
{
  USBPD_CABLEPLUG_FROMDFPUFP    = 0x00,                           /*!< Message originated from a DFP or UFP    */
  USBPD_PORTPOWERROLE_SNK       = USBPD_CABLEPLUG_FROMDFPUFP ,    /*!< Sink                                    */
  USBPD_CABLEPLUG_FROMCABLEPLUG = 0x01,                           /*!< Message originated from a Cable Plug    */
  USBPD_PORTPOWERROLE_SRC       = USBPD_CABLEPLUG_FROMCABLEPLUG,  /*!< Source                                  */
  
  USBPD_PORTPOWERROLE_DRP       = 0x4,                            /*!< DRP role                                */
  
  USBPD_PORTPOWERROLE_DRP_SNK   = USBPD_PORTPOWERROLE_SNK + USBPD_PORTPOWERROLE_DRP,   /*!< DR Device starting as Sink   */
  USBPD_PORTPOWERROLE_DRP_SRC   = USBPD_PORTPOWERROLE_SRC + USBPD_PORTPOWERROLE_DRP,   /*!< DR Device starting as Source */
  USBPD_PORTPOWERROLE_UNKNOWN   = 0xFF,
}USBPD_PortPowerRole_TypeDef;

/**
  * @brief  USB PD Data Message Types structure definition
  *
  */
typedef enum
{
  USBPD_DATAMSG_SRC_CAPABILITIES         = 0x01,  /*!< Source Capabilities Data Message  */
  USBPD_DATAMSG_REQUEST                  = 0x02,  /*!< Request Data Message              */
  USBPD_DATAMSG_BIST                     = 0x03,  /*!< BIST Data Message                 */
  USBPD_DATAMSG_SNK_CAPABILITIES         = 0x04,  /*!< Sink_Capabilities Data Message    */
  USBPD_DATAMSG_VENDOR_DEFINED           = 0x0F   /*!< Vendor_Defined Data Message       */
}USBPD_DataMsg_TypeDef;

/**
  * @brief  USB PD Control Message Types structure definition
  *
  */
typedef enum
{
  USBPD_CONTROLMSG_GOODCRC               = 0x01,  /*!< GoodCRC Control Message         */
  USBPD_CONTROLMSG_GOTOMIN               = 0x02,  /*!< GotoMin Control Message         */
  USBPD_CONTROLMSG_ACCEPT                = 0x03,  /*!< Accept Control Message          */
  USBPD_CONTROLMSG_REJECT                = 0x04,  /*!< Reject Control Message          */
  USBPD_CONTROLMSG_PING                  = 0x05,  /*!< Ping Control Message            */
  USBPD_CONTROLMSG_PS_RDY                = 0x06,  /*!< PS_RDY Control Message          */
  USBPD_CONTROLMSG_GET_SRC_CAP           = 0x07,  /*!< Get_Source_Cap Control Message  */
  USBPD_CONTROLMSG_GET_SNK_CAP           = 0x08,  /*!< Get_Sink_Cap Control Message    */
  USBPD_CONTROLMSG_DR_SWAP               = 0x09,  /*!< DR_Swap Control Message         */
  USBPD_CONTROLMSG_PR_SWAP               = 0x0A,  /*!< PR_Swap Control Message         */
  USBPD_CONTROLMSG_VCONN_SWAP            = 0x0B,  /*!< VCONN_Swap Control Message      */
  USBPD_CONTROLMSG_WAIT                  = 0x0C,  /*!< Wait Control Message            */
  USBPD_CONTROLMSG_SOFT_RESET            = 0x0D   /*!< Soft_Reset Control Message      */
}USBPD_ControlMsg_TypeDef;

/**
  * @brief  USB PD Extended Message Types structure definition
  *
  */
typedef enum
{
  USBPD_EXT_NONE                  = USBPD_EXTENDED_MESSAGE, 
  USBPD_EXT_SOURCE_CAPABILITIES   = (USBPD_EXTENDED_MESSAGE | 0x01),  /*!< sent by Source or Dual-Role Power    - SOP only  */
  USBPD_EXT_STATUS                = (USBPD_EXTENDED_MESSAGE | 0x02),  /*!< sent by Source                       - SOP only  */
  USBPD_EXT_GET_BATTERY_CAP       = (USBPD_EXTENDED_MESSAGE | 0x03),  /*!< sent by Source or Sink               - SOP only  */
  USBPD_EXT_GET_BATTERY_STATUS    = (USBPD_EXTENDED_MESSAGE | 0x04),  /*!< sent by Source or Sink               - SOP only  */
  USBPD_EXT_BATTERY_CAPABILITIES  = (USBPD_EXTENDED_MESSAGE | 0x05),  /*!< sent by Source or Sink               - SOP only  */
  USBPD_EXT_GET_MANUFACTURER_INFO = (USBPD_EXTENDED_MESSAGE | 0x06),  /*!< sent by Source or Sink or Cable Plug - SOP*      */
  USBPD_EXT_MANUFACTURER_INFO     = (USBPD_EXTENDED_MESSAGE | 0x07),  /*!< sent by Source or Sink or Cable Plug - SOP*      */
  USBPD_EXT_SECURITY_REQUEST      = (USBPD_EXTENDED_MESSAGE | 0x08),  /*!< sent by Source or Sink               - SOP*      */
  USBPD_EXT_SECURITY_RESPONSE     = (USBPD_EXTENDED_MESSAGE | 0x09),  /*!< sent by Source or Sink or Cable Plug - SOP*      */
  USBPD_EXT_FIRM_UPDATE_REQUEST   = (USBPD_EXTENDED_MESSAGE | 0x0A),  /*!< sent by Source or Sink               - SOP*      */
  USBPD_EXT_FIRM_UPDATE_RESPONSE  = (USBPD_EXTENDED_MESSAGE | 0x0B),  /*!< sent by Source or Sink or Cable Plug - SOP*      */
}USBPD_ExtendedMsg_TypeDef;

/**
  * @brief  USB PD BIST Mode Types structure definition
  *
  */
typedef enum
{
  USBPD_BIST_RECEIVER_MODE               = 0x00,  /*!< BIST Receiver Mode      */
  USBPD_BIST_TRANSMIT_MODE               = 0x01,  /*!< BIST Transmit Mode      */
  USBPD_RETURNED_BIST_COUNTERS           = 0x02,  /*!< Returned BIST Counters  */
  USBPD_BIST_CARRIER_MODE0               = 0x03,  /*!< BIST Carrier Mode 0     */
  USBPD_BIST_CARRIER_MODE1               = 0x04,  /*!< BIST Carrier Mode 1     */
  USBPD_BIST_CARRIER_MODE2               = 0x05,  /*!< BIST Carrier Mode 2     */
  USBPD_BIST_CARRIER_MODE3               = 0x06,  /*!< BIST Carrier Mode 3     */
  USBPD_BIST_EYE_PATTERN                 = 0x07,  /*!< BIST Eye Pattern        */
  USBPD_BIST_TEST_DATA                   = 0x08   /*!< BIST Test Data          */
}USBPD_BISTMsg_TypeDef;

/**
  * @brief  USB PD Source Fixed Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxCurrentIn10mAunits :
        10;
    uint32_t VoltageIn50mVunits :
        10;
    uint32_t PeakCurrent :
        2;
    uint32_t Reserved22_24 :
        3;
    uint32_t DataRoleSwap :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t ExternallyPowered :
        1;
    uint32_t USBSuspendSupported :
        1;
    uint32_t DualRolePower :
        1;
    uint32_t FixedSupply :
        2;
  }
  b;
}USBPD_SRCFixedSupplyPDO_TypeDef;

/**
  * @brief  USB PD Source Variable Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxCurrentIn10mAunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t VariableSupply :
        2;
  }
  b;
}USBPD_SRCVariableSupplyPDO_TypeDef;

/**
  * @brief  USB PD Source Battery Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxAllowablePowerIn250mWunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t Battery :
        2;
  }
  b;
}USBPD_SRCBatterySupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Fixed Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t OperationalCurrentIn10mAunits :
        10;
    uint32_t VoltageIn50mVunits :
        10;
    uint32_t Reserved20_24 :
        5;
    uint32_t DataRoleSwap :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t ExternallyPowered :
        1;
    uint32_t HigherCapability :
        1;
    uint32_t DualRolePower :
        1;
    uint32_t FixedSupply :
        2;
  }
  b;
}USBPD_SNKFixedSupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Variable Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t OperationalCurrentIn10mAunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t VariableSupply :
        2;
  }
  b;
}USBPD_SNKVariableSupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Battery Supply Power Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t OperationalPowerIn250mWunits :
        10;
    uint32_t MinVoltageIn50mVunits :
        10;
    uint32_t MaxVoltageIn50mVunits :
        10;
    uint32_t Battery :
        2;
  }
  b;
}USBPD_SNKBatterySupplyPDO_TypeDef;

/**
  * @brief  USB PD Sink Fixed and Variable Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxOperatingCurrent10mAunits : /* Corresponding to min if GiveBackFlag = 1 */
        10;
    uint32_t OperatingCurrentIn10mAunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKFixedVariableRDO_TypeDef;


/**
  * @brief  USB PD Sink Fixed and Variable Request Data Object with GiveBack Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MinOperatingCurrent10mAunits :
        10;
    uint32_t OperatingCurrentIn10mAunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKFixedVariableRDOGiveBack_TypeDef;


/**
  * @brief  USB PD Sink Battery Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MaxOperatingPowerIn250mWunits :
        10;
    uint32_t OperatingPowerIn250mWunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKBatteryRDO_TypeDef;


/**
  * @brief  USB PD Sink Battery with GiveBack Request Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {
    uint32_t MinOperatingPowerIn250mWunits :
        10;
    uint32_t OperatingPowerIn250mWunits :
        10;
    uint32_t Reserved20_23 :
        4;
    uint32_t NoUSBSuspend :
        1;
    uint32_t USBCommunicationsCapable :
        1;
    uint32_t CapabilityMismatch :
        1;
    uint32_t GiveBackFlag :
        1;
    uint32_t ObjectPosition :
        3;
    uint32_t Reserved31 :
        1;
  }
  b;
}USBPD_SNKBatteryGiveBackRDO_TypeDef;

/**
  * @brief  USBPD Port PDO Structure definition
  *
  */
typedef struct
{
  uint32_t *ListOfPDO;                          /*!< Pointer on Power Data Objects list, defining 
                                                     port capabilities */
  uint8_t  NumberOfPDO;                         /*!< Number of Power Data Objects defined in ListOfPDO 
                                                     This parameter must be set at max to USBPD_DEF_NBMAX_PDO value */
}USBPD_PortPDO_TypeDef;

/**
  * @brief  USB PD BIST Data Object Structure definition
  *
  */
typedef union
{
  uint32_t d32;
  struct
  {  
    uint32_t BistErrorCounter :
      16;
    uint32_t Reserved16_27 :
      12;
    uint32_t BistMode : 
      4;
  }
  b;
}USBPD_BISTDataObject_TypeDef;

/** @brief  Sink requested power profile Structure definition
  *
  */
typedef struct
{
  uint32_t MaxOperatingCurrentInmAunits;           /*!< Sink board Max operating current in mA units   */
  uint32_t OperatingVoltageInmVunits;              /*!< Sink board operating voltage in mV units       */
  uint32_t MaxOperatingVoltageInmVunits;           /*!< Sink board Max operating voltage in mV units   */
  uint32_t MinOperatingVoltageInmVunits;           /*!< Sink board Min operating voltage in mV units   */
  uint32_t OperatingPowerInmWunits;                /*!< Sink board operating power in mW units         */
  uint32_t MaxOperatingPowerInmWunits;             /*!< Sink board Max operating power in mW units     */
}USBPD_SNKPowerRequest_TypeDef;

/** @defgroup USBPD_CORE_PDO_Exported_Types USBPD Core PDO related Exported Types
  * @{
  */ 

/** @defgroup USBPD_CORE_PDO_Type_TypeDef PDO type definition
  * @brief  PDO type values in PDO definition 
  * @{
  */

typedef enum {
  USBPD_CORE_PDO_TYPE_FIXED = 0x00,                 /*!< Fixed Supply PDO                             */
  USBPD_CORE_PDO_TYPE_BATTERY = 0x01,               /*!< Battery Supply PDO                           */
  USBPD_CORE_PDO_TYPE_VARIABLE = 0x02,              /*!< Variable Supply (non-battery) PDO            */
} USBPD_CORE_PDO_Type_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_DRPowerSupport_TypeDef DRP Support type 
  * @brief  DRP support values in PDO definition (Source or Sink)
  * @{
  */
typedef enum {
  USBPD_CORE_PDO_DRP_NOT_SUPPORTED = 0x00,          /*!< Dual Role Power not supported                */
  USBPD_CORE_PDO_DRP_SUPPORTED     = 0x01,          /*!< Dual Role Power supported                    */
} USBPD_CORE_PDO_DRPowerSupport_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_USBSuspendSupport_TypeDef USB Suspend type 
  * @brief  USB Suspend support values in PDO definition (Source)
  * @{
  */
typedef enum {
  USBPD_CORE_PDO_USBSUSP_NOT_SUPPORTED = 0x00,      /*!< USB Suspend not supported                    */
  USBPD_CORE_PDO_USBSUSP_SUPPORTED     = 0x01,      /*!< USB Suspend supported                        */
} USBPD_CORE_PDO_USBSuspendSupport_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_ExtPowered_TypeDef Externally Powered type 
  * @brief  Fixed Power Source Externally Powered indication values in PDO definition (Source or Sink)
  * @{
  */
typedef enum {
  USBPD_CORE_PDO_NOT_EXT_POWERED = 0x00,            /*!< No external power source is available        */
  USBPD_CORE_PDO_EXT_POWERED     = 0x01,            /*!< External power source is available           */
} USBPD_CORE_PDO_ExtPowered_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_USBCommCapable_TypeDef USB Communication capability type 
  * @brief  USB Communication capability over USB Data lines indication values in PDO definition (Source or Sink)
  * @{
  */
typedef enum {
  USBPD_CORE_PDO_USBCOMM_NOT_CAPABLE = 0x00,        /*!< Device not capable of communication over USB Data lines */
  USBPD_CORE_PDO_USBCOMM_CAPABLE     = 0x01,        /*!< Device capable of communication over USB Data lines     */
} USBPD_CORE_PDO_USBCommCapable_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_DRDataSupport_TypeDef Dual Role Data Support type 
  * @brief  Dual Role Data support values in PDO definition (Source or Sink)
  * @{
  */
typedef enum {
  USBPD_CORE_PDO_DRD_NOT_SUPPORTED = 0x00,          /*!< Dual Role Data not supported                 */
  USBPD_CORE_PDO_DRD_SUPPORTED     = 0x01,          /*!< Dual Role Data supported                     */
} USBPD_CORE_PDO_DRDataSupport_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_PeakCurr_TypeDef Peak Current Capability type
  * @brief  Fixed Power Source Peak Current Capability type structure definition (Source)
  * @{
  */
typedef enum
{
  USBPD_CORE_PDO_PEAKEQUAL = 0x00,                  /*!< Peak current equals                          */
  USBPD_CORE_PDO_PEAKOVER1 = 0x01,                  /*!< Overload Capabilities:
  1. Peak current equals 150% IOC for 1ms @ 5% duty cycle (low current equals 97% IOC for 19ms)
  2. Peak current equals 125% IOC for 2ms @ 10% duty cycle (low current equals 97% IOC for 18ms)
  3. Peak current equals 110% IOC for 10ms @ 50% duty cycle (low current equals 90% IOC for 10ms */
  USBPD_CORE_PDO_PEAKOVER2 = 0x02,                  /*!< Overload Capabilities:
  1. Peak current equals 200% IOC for 1ms @ 5% duty cycle (low current equals 95% IOC for 19ms)
  2. Peak current equals 150% IOC for 2ms @ 10% duty cycle (low current equals 94% IOC for 18ms)
  3. Peak current equals 125% IOC for 10ms @ 50% duty cycle (low current equals 75% IOC for 10ms)*/
  USBPD_CORE_PDO_PEAKOVER3 = 0x03,                  /*!< Overload Capabilities:
  1. Peak current equals 200% IOC for 1ms @ 5% duty cycle (low current equals 95% IOC for 19ms)
  2. Peak current equals 175% IOC for 2ms @ 10% duty cycle (low current equals 92% IOC for 18ms)
  3. Peak current equals 150% IOC for 10ms @ 50% duty cycle (low current equals 50% IOC for 10ms)*/
}USBPD_CORE_PDO_PeakCurr_TypeDef;
/**
  * @}
  */

/** @defgroup USBPD_CORE_PDO_HigherCapability_TypeDef USB Higher Capability type 
  * @brief  Values in PDO definition (Sink) indicating if Sink needs more than vSafe5V to provide full functionality
  * @{
  */
typedef enum {
  USBPD_CORE_PDO_NO_HIGHER_CAPABILITY  = 0x00,      /*!< No need for more than vSafe5V to provide full functionality */
  USBPD_CORE_PDO_HIGHER_CAPABILITY     = 0x01,      /*!< Sink needs more than vSafe5V to provide full functionality  */
} USBPD_CORE_PDO_HigherCapability_TypeDef; 
/**
  * @}
  */

/** @defgroup USBPD_CORE_DataInfoType_TypeDef USB Core Data information type
  * @brief Data Infor types used in PE callbacks (USBPD_PE_GetDataInfo and USBPD_PE_SetDataInfo)
  * @{
  */
typedef enum {
  USBPD_CORE_DATATYPE_SRC_PDO          = 0x00,      /*!< Handling of port Source PDO */
  USBPD_CORE_DATATYPE_SNK_PDO          = 0x01,      /*!< Handling of port Sink PDO */
  USBPD_CORE_DATATYPE_RDO_POSITION     = 0x02,      /*!< Storage of requested DO position in PDO list */
  USBPD_CORE_DATATYPE_REQ_VOLTAGE      = 0x03,      /*!< Storage of requested voltage value */
  USBPD_CORE_DATATYPE_RCV_SRC_PDO      = 0x04,      /*!< Storage of Received Source PDO values */
  USBPD_CORE_DATATYPE_RCV_SNK_PDO      = 0x05,      /*!< Storage of Received Sink PDO values */
  USBPD_CORE_DATATYPE_RCV_REQ_PDO      = 0x06,      /*!< Storage of Received Sink Request PDO value */
  USBPD_CORE_DATATYPE_REQUEST_DO       = 0x07,      /*!< Storage of DO to be used in request message (from Sink to Source) */
} USBPD_CORE_DataInfoType_TypeDef; 

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USBPD_CORE_DEF_Exported_Macros USBPD Core DEF Exported Macros
  * @{
  */
/**
  * @brief  Compare two variables and return the smallest
  * @param  __VAR1__ First variable to be compared
  * @param  __VAR2__ Second variable to be compared
  * @retval Returns the smallest variable
  */
#define USBPD_MIN(__VAR1__, __VAR2__) (((__VAR1__) <= (__VAR2__))?(__VAR1__):(__VAR2__))

/**
  * @brief  Check if the requested voltage is valid
  * @param  __MV__    Requested voltage in mV units 
  * @param  __MAXMV__ Max Requested voltage in mV units
  * @param  __MINMV__ Min Requested voltage in mV units
  * @retval 1 if valid voltage else 0
  */
#define USBPD_IS_VALID_VOLTAGE(__MV__, __MAXMV__, __MINMV__) ((((__MV__) <= (__MAXMV__)) && ((__MV__) >= (__MINMV__)))? 1: 0)

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/


/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_DEF_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
