/**
  ******************************************************************************
  * @file    usbpd_phy.c
  * @author  MCD Application Team
  * @brief   This file contains PHY layer functions.
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

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "usbpd_phy.h"
#include "usbpd_hw_if.h"
#include "usbpd_cad_hw_if.h"
#include "usbpd_pwr_if.h"
#include <string.h>
/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_PHY
  * @brief   This file contains PHY layer functions.
  * @details Receive from PRL a message and create a structured packet (according to the USBPD specifications):
  *          <SOP><DATA:[<HEADER><DATAOBJECTS>]><CRC><EOP>
  * @{
  */

/* Private defines -----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_defines USBPD DEVICE PHY Private defines
  * @brief PHY internally used defines
  * @{
  */
#define __TX_BUFF_SIZE ((uint16_t)48)                                 /*!< Transmission buffer size (bytes) */
#define __TX_BUFF_BITSIZE_MAX ((uint16_t)(__TX_BUFF_SIZE * ((uint16_t)8)))  /*!< Transmission buffer bitsize */
#define __RX_DATA_LEN 40                      /*!< size of the incoming buffer */

/**
  * @}
  */

/* Private typedef -----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_typedef USBPD DEVICE PHY Private typedef
  * @brief Structures and enums internally used by the PHY layer
  * @{
  */
/**
  * @brief Private struct to store buffers and Tx variables
  */
#ifdef __STAT
typedef struct
{
  uint32_t TxCount;
  uint32_t TxBusy;
  uint32_t TxSent;
  uint32_t TxError;
  uint32_t ResetCount;
  uint32_t ResetError;

  /* Rx stats */
  uint32_t RxSopCorrected;
  uint32_t RX2Count;
  uint32_t RX2Special;
  uint32_t RX2Ok;
  uint32_t RX2Error;
  uint32_t RX2ErrorEOP;
  uint32_t RX2ErrorCRC;
  uint32_t RX2ErrorUnsupportedSOP;
  uint32_t RX2ErrorInvalidSOP;
  uint32_t RX2ErrorInvalidSymbol;

} PHY_StatsTypeDef;
#endif

/**
  * @brief Status available for the PHY layer
  */
typedef enum
{
  PHY_StateNone         = 0,  /*!< PHY State Not available              */
  PHY_StateWaitingIdle  = 1,  /*!< PHY State Idle                       */
  PHY_StateBusy         = 2,  /*!< PHY State Busy                       */
  PHY_StateBusyTxStart  = 3,  /*!< PHY State Busy, transmission process */
  PHY_StateBusyRx       = 4,  /*!< PHY State Busy, reception process    */
  PHY_StateBusyBIST_Tx  = 5,  /*!< PHY State Busy, TX BIST              */
  PHY_StateBusyBIST_Rx  = 6,  /*!< PHY State Busy, RX BIST              */
} PHY_State;

/**
 * @brief structure to support the decoding on fly during the reception
 */
typedef struct
{
  USBPD_PHY_RX_Status_TypeDef Status;       /*!< Status of the reception process   */
  uint32_t OrderSet;                        /*!< Last orderset received            */
  USBPD_SOPType_TypeDef MsgType;            /*!< Type of the incoming message      */
  uint8_t Data[__RX_DATA_LEN];              /*!< buffer for the incoming message   */
  uint32_t DataCount;                       /*!< Counting of received data         */
} PHY_RxDecodingTypeDef;

/**
  * @brief Handle to support the data of the layer
  */
typedef struct
{
  USBPD_PHY_Callbacks  *cbs;                /*!< callbacks of the @ref USBPD_CORE_PRL */
  uint8_t*   pRxBuffer;                     /*!< buffer provided by the @ref USBPD_CORE_PRL */
  PHY_State  State;                         /*!< Current state of the PHY layer @ref PHY_State */
  uint32_t   TxBuffer[__TX_BUFF_SIZE/4];    /*!< Temporary TX buffer in 5B coding (32 bit size), no BMC coding, ready for USBPD_HW_IF */
  uint32_t   TxDatabitLen;                  /*!< TX buffer len (bits) */
  PHY_RxDecodingTypeDef RxDec;              /*!< var to support the RX decoding */
  uint32_t   SupportedSOP;                  /*!< bit field SOP"Debug SOP'Debug SOP" SOP' SOP */
  uint32_t   TxCount;                         /*!< TX count, for debug purpose */
#ifdef __STAT
  PHY_StatsTypeDef Stats;                     /*!< Statistics (Only for debug scope) */
#endif
} PHY_HandleTypeDef;

/**
  * @brief prototype definition shared in several callbacks
  */
typedef void (*PHY_CB_t)(uint8_t PortNum, USBPD_SOPType_TypeDef Type); 

/**
  * @}
  */

/* Private define and macro --------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_macros USBPD DEVICE PHY Private macros
  * @brief defines and macros to support the coding/decoding through lookup table and bit operations
  * @{
  */
#define __SIZE     ((uint8_t)4)               /*!< Size in bytes of the coding/decoding variables */
#define __SIZEBIT   ((uint16_t)((__SIZE)*8))  /*!< Size in bits of the coding/decoding variables */
#define __BITMASK(__VAL__) ((1<<(__VAL__))-1) /*!< Create a n bit mask i.e. n=5 => 0x1F */

/**
  * @}
  */

/** @defgroup USBPD_DEVICE_PHY_Key_Code K-Codes 
  * @brief  K-Codes available in the Symbol Encoding Table
  * @details  K-Codes available in the Symbol Encoding Table, according to the USBPD specifications
  * @{
  */
#define KC_S_SYNC1 0x18 /*!< Startsynch #1 */
#define KC_S_SYNC2 0x11 /*!< Startsynch #2 */
#define KC_S_SYNC3 0x06 /*!< Startsynch #3 */
#define KC_S_RST1  0x07 /*!< Hard Reset #1 */
#define KC_S_RST2  0x19 /*!< Hard Reset #2 */
#define KC_S_EOP   0x0D /*!< EOP End Of Packet */
#define KC_BITSIZE 0x05 /*!< Size of in bits of the K-Code */
/**
  * @}
  */

#define SYM_FOR_WORD        6  /*!< Number of symbols inside a 32bit word for coding/decoding */

/** @defgroup USBPD_DEVICE_PHY_Order_Set Order Set definition and tools
  * @brief  Tools for ordersets and defined items
  * @details  Tools to  create the ordersets by the K-Codes and definition, refer to the USBPD specifications
  * @{
  */

/**
  * @brief Orderset creation through the 4 K-Codes
  */
#define OS_MAKE(_KC1_, _KC2_, _KC3_, _KC4_)   ((_KC1_) | ((_KC2_)<<5) | ((_KC3_)<<10) | ((_KC4_)<<15))
#define OS_KC_NUM   4                                                               /*!< Num of K-Code in a Orderset */
#define OS_BITSIZE ((KC_BITSIZE) * (OS_KC_NUM))                                     /*!< Orderset size in bits */
#define OS_MASK (__BITMASK(OS_BITSIZE))                                             /*!< Orderset mask, 20 bits => 0xFFFFF */
#define OS_NUM              7                                                       /*!< Number of predefined Orderedsets */
#define OS_SYM_SOP         OS_MAKE(KC_S_SYNC1, KC_S_SYNC1, KC_S_SYNC1, KC_S_SYNC2)  /*!< SOP : Start of Packet Sequence */
#define OS_SYM_SOP_1       OS_MAKE(KC_S_SYNC1, KC_S_SYNC1, KC_S_SYNC3, KC_S_SYNC3)  /*!< Prime SOP : Start of Packet Sequence Prime */
#define OS_SYM_SOP_2       OS_MAKE(KC_S_SYNC1, KC_S_SYNC3, KC_S_SYNC1, KC_S_SYNC3)  /*!< Double SOP : Start of Packet Sequence Double */
#define OS_SYM_SOP_DBG_1   OS_MAKE(KC_S_SYNC1, KC_S_RST2,  KC_S_RST2,  KC_S_SYNC3)  /*!< Prime Debug SOP : Start of Packet Sequence Prime Debug */
#define OS_SYM_SOP_DBG_2   OS_MAKE(KC_S_SYNC1, KC_S_RST2,  KC_S_SYNC3, KC_S_SYNC2)  /*!< Double Debug SOP : Start of Packet Sequence Double Debug */
#define OS_SYM_HARD_RESET  OS_MAKE(KC_S_RST1,  KC_S_RST1,  KC_S_RST1,  KC_S_RST2 )  /*!< Hard Reset */
#define OS_SYM_CABLE_RESET OS_MAKE(KC_S_RST1,  KC_S_SYNC1, KC_S_RST1,  KC_S_SYNC3)  /*!< Cable Reset */
/**
  * @}
  */

#define CODE_5B_INVALID 0xFF                                                    /*!< Invalid symbol */
#define CODE_5B_20BIT_INVALID 0xFFFFFFFF                                        /*!< Invalid group symbols */
#define CODE_5B_20BIT_IS_INVALID(__VAL__) ((__VAL__) == CODE_5B_20BIT_INVALID)  /*!< Group symbols validation  */

/* Bit Sizes */
#define CODE_5B_ITEM1_BITSIZE KC_BITSIZE             /*!< Symbol size in bits, the same of @ref KC_BITSIZE */
#define CODE_5B_ITEM1_MASK (__BITMASK(CODE_5B_ITEM1_BITSIZE))   /*!< Symbol mask, 5 bits => 0x1F */
#define CODE_5B_ITEM2_BITSIZE (CODE_5B_ITEM1_BITSIZE * 2)     /*!< Two-Symbol size in bits */
#define CODE_5B_ITEM4_BITSIZE (CODE_5B_ITEM1_BITSIZE * 4)     /*!< Four-Symbol size in bits */

/* allowed size of the 5B packet (according to the USBPD) */
#define FRAME_5B_BitSizeIsValid(__BS__) (\
                          ((__BS__) == 20 ) || \
                          ((__BS__) == 85 ) || \
                          ((__BS__) == 125) || \
                          ((__BS__) == 165) || \
                          ((__BS__) == 205) || \
                          ((__BS__) == 245) || \
                          ((__BS__) == 285) || \
                          ((__BS__) == 325) || \
                          ((__BS__) == 365) \
                          )

/**
  * @}
  */

/* Private variables ---------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_variables USBPD DEVICE PHY Private variables
  * @brief PHY variable and const, 
  * @details In this section there are the lookup tables: @ref coding4b5b and @ref decoding5b4b
  *          useful to improve the performance of the coding/decoding processes
  * @{
  */
/* Ordered Sets */
static const uint32_t OrderSets[OS_NUM] =
{
  OS_SYM_SOP          , /*!<  SOP*  MESSAGES       */
  OS_SYM_SOP_1        , /*!<  SOP'  MESSAGES       */
  OS_SYM_SOP_2        , /*!<  SOP'' MESSAGES       */
  OS_SYM_SOP_DBG_1    , /*!<  SOP'  DEBUG_MESSAGES */
  OS_SYM_SOP_DBG_2    , /*!<  SOP'' DEBUG_MESSAGES */
  OS_SYM_HARD_RESET   , /*!<  HARD RESET MESSAGE   */
  OS_SYM_CABLE_RESET  , /*!<  CABLE RESET MESSAGE  */
};

/* Lookup Table with 4b/5b + BMC encoding */
static const uint8_t coding4b5b[] =
{
  /* 0 = 0000 */ 0x1E /* 11110 */,
  /* 1 = 0001 */ 0x09 /* 01001 */,
  /* 2 = 0010 */ 0x14 /* 10100 */,
  /* 3 = 0011 */ 0x15 /* 10101 */,
  /* 4 = 0100 */ 0x0A /* 01010 */,
  /* 5 = 0101 */ 0x0B /* 01011 */,
  /* 6 = 0110 */ 0x0E /* 01110 */,
  /* 7 = 0111 */ 0x0F /* 01111 */,
  /* 8 = 1000 */ 0x12 /* 10010 */,
  /* 9 = 1001 */ 0x13 /* 10011 */,
  /* A = 1010 */ 0x16 /* 10110 */,
  /* B = 1011 */ 0x17 /* 10111 */,
  /* C = 1100 */ 0x1A /* 11010 */,
  /* D = 1101 */ 0x1B /* 11011 */,
  /* E = 1110 */ 0x1C /* 11100 */,
  /* F = 1111 */ 0x1D /* 11101 */,
};

/* Lookup Table with 5b/4b dencoding */
static const uint8_t decoding5b4b[] =
{
  /* 00 = 00000 */ CODE_5B_INVALID /* E--- */,
  /* 01 = 00001 */ CODE_5B_INVALID /* E--- */,
  /* 02 = 00010 */ CODE_5B_INVALID /* E--- */,
  /* 03 = 00011 */ CODE_5B_INVALID /* E--- */,
  /* 04 = 00100 */ CODE_5B_INVALID /* E--- */,
  /* 05 = 00101 */ CODE_5B_INVALID /* E--- */,
  /* 06 = 00110 */ CODE_5B_INVALID /* E--- */,
  /* 07 = 00111 */ CODE_5B_INVALID /* E--- */,
  /* 08 = 01000 */ CODE_5B_INVALID /* E--- */,
  /* 09 = 01001 */ 0x01 /* 0001 */,
  /* 0A = 01010 */ 0x04 /* 0100 */,
  /* 0B = 01011 */ 0x05 /* 0101 */,
  /* 0C = 01100 */ CODE_5B_INVALID /* E--- */,
  /* 0D = 01101 */ CODE_5B_INVALID /* E--- */,
  /* 0E = 01110 */ 0x06 /* 0110 */,
  /* 0F = 01111 */ 0x07 /* 0111 */,
  /* 10 = 10000 */ CODE_5B_INVALID /* E--- */,
  /* 11 = 10001 */ CODE_5B_INVALID /* E--- */,
  /* 12 = 10010 */ 0x08 /* 1000 */,
  /* 13 = 10011 */ 0x09 /* 1001 */,
  /* 14 = 10100 */ 0x02 /* 0010 */,
  /* 15 = 10101 */ 0x03 /* 0011 */,
  /* 16 = 10110 */ 0x0A /* 1010 */,
  /* 17 = 10111 */ 0x0B /* 1011 */,
  /* 18 = 11000 */ CODE_5B_INVALID /* E--- */,
  /* 19 = 11001 */ CODE_5B_INVALID /* E--- */,
  /* 1A = 11010 */ 0x0C /* 1100 */,
  /* 1B = 11011 */ 0x0D /* 1101 */,
  /* 1C = 11100 */ 0x0E /* 1110 */,
  /* 1D = 11101 */ 0x0F /* 1111 */,
  /* 1E = 11110 */ 0x00 /* 0000 */,
  /* 1F = 11111 */ CODE_5B_INVALID /* E--- */,
};

/** Internal struct for RXTX */
static PHY_HandleTypeDef PHY_Ports[USBPD_PORT_COUNT];

/**
  * @}
  */

/* Private function prototypes -----------------------------------------------*/
/** @defgroup USBPD_DEVICE_PHY_Private_functions USBPD DEVICE PHY Private functions
  * @{
  */
void                        PHY_Stat_Reset(uint8_t PortNum);
USBPD_StatusTypeDef         PHY_PortInit(uint8_t PortNum, USBPD_PHY_Callbacks *cbs, uint8_t *pRxBuffer, uint32_t SupportedSOP);
USBPD_StatusTypeDef         PHY_PreparePacket(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pPacketBuffer, uint8_t Size);
void                        PHY_TxBuffer_Reset(uint8_t PortNum);
void                        PHY_BistCompleted(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode);
void                        PHY_TxCompleted(uint8_t portnum);
USBPD_SOPType_TypeDef       PHY_SopDetect(uint8_t PortNum, uint32_t OrderSet);
void                        PHY_TxBuffer_Append(uint8_t PortNum, uint32_t val, uint8_t nbit);
void                        USBPD_PHY_ResetCompleted(uint8_t PortNum, USBPD_SOPType_TypeDef Type);
USBPD_PHY_RX_Status_TypeDef PHY_Rx_Reset(uint8_t PortNum);
USBPD_PHY_RX_Status_TypeDef PHY_Rx_Accumulate(uint8_t PortNum, uint32_t data); /* 10 bits => 1 byte */
USBPD_PHY_RX_Status_TypeDef PHY_Rx_Completed(uint8_t PortNum);
#ifdef __STAT
void PHY_Stat_Reset(uint8_t PortNum);
#endif /* __STAT */

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/

/** @defgroup USBPD_DEVICE_PHY_Exported_Functions USBPD DEVICE PHY Exported functions
  * @{
  */
/**
  * @brief  Initialize the PHY of a specified port.
  * @param  PortNum       Number of the port.
  * @param  pCallbacks    PHY callbacks
  * @param  pRxBuffer     Buffer to storage received message.
  * @param  PowerRole     Power Role of the board.
  * @param  SupportedSOP  Supported SOP
  * @retval USBPD_StatusTypeDef status
  */
USBPD_StatusTypeDef USBPD_PHY_Init(uint8_t PortNum, USBPD_PHY_Callbacks *pCallbacks, uint8_t *pRxBuffer, USBPD_PortPowerRole_TypeDef PowerRole, uint32_t SupportedSOP)
{
  /* set all callbacks */
  USBPD_HW_IF_Callbacks hwif_cbs;
  hwif_cbs.USBPD_HW_IF_TxCompleted    = PHY_TxCompleted;
  hwif_cbs.USBPD_HW_IF_BistCompleted  = PHY_BistCompleted;
  hwif_cbs.USBPD_HW_IF_RX_Reset       = PHY_Rx_Reset;
  hwif_cbs.USBPD_HW_IF_RX_Accumulate  = PHY_Rx_Accumulate;
  hwif_cbs.USBPD_HW_IF_RX_Completed   = PHY_Rx_Completed;

  /* Initialize the hardware for the port */
  USBPD_HW_IF_PortHwInit(PortNum, hwif_cbs, PowerRole);

  /* Initialize port related functionalities inside this layer */
  if (PHY_PortInit(PortNum, pCallbacks, pRxBuffer, SupportedSOP))
  {
    return USBPD_ERROR;
  }

#ifdef __STAT
  PHY_Stat_Reset(PortNum);
#endif

  return USBPD_OK;
}

/**
  * @brief  return the retry counter value in us.
  * @param  PortNum    Number of the port.
  * @retval retry counter value in us.
  */
uint32_t USBPD_PHY_GetRetryTimerValue(uint8_t PortNum)
{
  return 920u;
}

/**
  * @brief  Reset the PHY of a specified port.
  * @param  PortNum    Number of the port.
  * @retval None
  */
void USBPD_PHY_Reset(uint8_t PortNum)
{
  /* reset PHY layer */
  PHY_TxBuffer_Reset(PortNum);
}

/**
  * @brief  Request a Reset on a specified port.
  * @param  PortNum    Number of the port
  * @param  Type    The type of reset (hard or cable reset).
  * @retval HAL status
  */
USBPD_StatusTypeDef USBPD_PHY_ResetRequest(uint8_t PortNum, USBPD_SOPType_TypeDef Type)
{
  /* Send the requested reset */
#ifdef __STAT
  PHY_Ports[PortNum].Stats.ResetCount++;
#endif
  if (USBPD_PHY_SendMessage(PortNum, Type, NULL, 0) != USBPD_OK)
  {
#ifdef __STAT
    PHY_Ports[PortNum].Stats.ResetError++;
#endif
    return USBPD_ERROR;
  }

  PHY_Ports[PortNum].State = PHY_StateBusy;

  /* reset PHY layer */
  PHY_TxBuffer_Reset(PortNum);
  /* reset HW_IF layer */
  /* USBPD_HW_IF_Reset(PortNum, REQUEST); */

  /* Send reset information to PRL layer */
  if (PHY_Ports[PortNum].cbs->USBPD_PHY_ResetCompleted != NULL)
  {
    PHY_Ports[PortNum].cbs->USBPD_PHY_ResetCompleted(PortNum, Type);
  }

  PHY_Ports[PortNum].State = PHY_StateNone;

  return USBPD_OK;
}

/**
  * @brief  Send a Message.
  * @param  PortNum     Number of the port
  * @param  Type      Type of the message
  * @param  pBuffer      Pointer to the buffer to be transmitted
  * @param  Size      Size of the buffer (bytes)
  * @retval USBPD_StatusTypeDef status
  */
USBPD_StatusTypeDef USBPD_PHY_SendMessage(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer, uint16_t Size)
{
  USBPD_StatusTypeDef res = USBPD_FAIL;
#ifdef __STAT
  PHY_Ports[PortNum].Stats.TxCount++;
#endif

  if (PHY_Ports[PortNum].State != PHY_StateNone)
  {
#ifdef __STAT
    PHY_Ports[PortNum].Stats.TxBusy++;
#endif

#ifdef __BLOCKING_SENDMSG
    //while (/* PHY_Ports[PortNum].State != PHY_StateNone && */ !HW_StatusIdle(PortNum));
#else
    return USBPD_BUSY;
#endif
  }

  /* Prepare the packet to be sent, structure: <SOP><DATA><CRC><EOP> */
  res = PHY_PreparePacket(PortNum, Type, pBuffer, Size);

  /* if ok pass the packet to the HW_IF to be sent */
  if (res == USBPD_OK)
  {
    /* Setup the State of the port */
    PHY_Ports[PortNum].State = PHY_StateBusyTxStart;
    res = USBPD_HW_IF_SendBuffer(PortNum, (uint8_t *)PHY_Ports[PortNum].TxBuffer,  PHY_Ports[PortNum].TxDatabitLen);
  }

#ifdef __STAT
  if (USBPD_OK == res)
  {
    PHY_Ports[PortNum].Stats.TxSent++;
  }
  else
  {
    PHY_Ports[PortNum].Stats.TxError++;
  }
#endif
  PHY_Ports[PortNum].State = PHY_StateNone;

  return res;
}

/**
  * @brief  Send BIST pattern.
  * @param  PortNum    Number of the port
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PHY_Send_BIST_Pattern(uint8_t PortNum)
{
  USBPD_StatusTypeDef res = USBPD_OK;

  /* Setup the State of the port */
  PHY_Ports[PortNum].State = PHY_StateBusyBIST_Tx;

  /* Call the low-level function (HW_IF) to accomplish the BIST Carrier Mode Transmission */
  USBPD_HW_IF_Send_BIST_Pattern(PortNum);

  /* reset the status of the port */
  PHY_Ports[PortNum].State = PHY_StateNone;
  return res;
}

/**
  * @brief  Request PHY to exit of BIST mode 2
  * @param  PortNum   port number value
  * @param  mode      SOP BIST MODE 2
  * @retval USBPD status
  */
USBPD_StatusTypeDef USBPD_PHY_ExitTransmit(uint8_t PortNum, USBPD_SOPType_TypeDef mode)
{
  UNUSED(PortNum);
  UNUSED(mode);

  return USBPD_OK;
}

/**
  * @brief  Reset completed notification.
  * @param  PortNum   Number of the port
  * @param  Type      PD Type
  * @retval None
  */
void USBPD_PHY_ResetCompleted(uint8_t PortNum, USBPD_SOPType_TypeDef Type)
{
  PHY_Ports[PortNum].State = PHY_StateBusy;

  /* perform a PHY layer reset */
  USBPD_PHY_Reset(PortNum);

  /* notify to upper level (PRL) */
  if (PHY_Ports[PortNum].cbs->USBPD_PHY_ResetCompleted != NULL)
  {
    PHY_Ports[PortNum].cbs->USBPD_PHY_ResetCompleted(PortNum, Type);
  }

  /* reset the status of the port */
  PHY_Ports[PortNum].State = PHY_StateNone;
}

/**
  * @brief  Set the SinkTxNg value of the resistor, used in the collision avoidance
  * @param  PortNum  Number of the port
  * @retval None
  */
void USBPD_PHY_SetResistor_SinkTxNG(uint8_t PortNum)
{
}

/**
  * @brief  function to set the SinkTxOK
  * @param  PortNum  Number of the port
  * @retval none.
  */
void USBPD_PHY_SetResistor_SinkTxOK(uint8_t PortNum)
{
}

/**
  * @brief  function to set the supported SOP
  * @param  PortNum  Number of the port.
  * @param  List of the supported SOP
  * @retval None.
  */
void USBPD_PHY_SOPSupported(uint8_t PortNum,uint32_t SOPSupported)
{
  PHY_Ports[PortNum].SupportedSOP = SOPSupported;
}

/**
  * @brief  Check if SinkTxOK is set or not
  * @param  PortNum  Number of the port.
  * @retval USBPD_TRUE or USBPD_FALSE
  */
uint8_t USBPD_PHY_IsResistor_SinkTxOk(uint8_t PortNum)
{

  return ((vRd_1_5A == CAD_GetRPValue(PortNum)) ? USBPD_FALSE : USBPD_TRUE);
}

/**
 * @brief  function to set the SinkTxOK
 * @param  PortNum  Number of the port.
 * @retval USBPD_TRUE USBPD_FALSE.
  */
void USBPD_PHY_FastRoleSwapSignalling(uint8_t PortNum)
{
}

/**
  * @}
  */

/** @addtogroup USBPD_DEVICE_PHY_Private_functions
  * @brief PHY internally used functions
  * @{
  */

/**
  * @brief  Port initiatlization
  * @param  PortNum       Number of the port
  * @param  pCallback     PHY callbacks 
  * @param  pRxBuffer     Pointer on the reception buffer
  * @param  SupportedSOP  Supported SOP
  * @retval USBPD Status
  */
USBPD_StatusTypeDef PHY_PortInit(uint8_t PortNum, USBPD_PHY_Callbacks *pCallback, uint8_t *pRxBuffer, uint32_t SupportedSOP)
{
  /* Associate the RXBUF */
  PHY_Ports[PortNum].pRxBuffer    = pRxBuffer;
  PHY_Ports[PortNum].SupportedSOP = SupportedSOP;

  /* Associate the callbacks */
  PHY_Ports[PortNum].cbs = pCallback;
  PHY_TxBuffer_Reset(PortNum);
  PHY_Ports[PortNum].State = PHY_StateNone;

  return USBPD_OK;
}

/**
  * @brief  PRL Hard Reset indication from PHY
  * @param  PortNum Port Number of the port
  * @retval None
  */
void PHY_TxBuffer_Reset(uint8_t PortNum)
{
  /* Clean the buffer*/
  memset((void *)PHY_Ports[PortNum].TxBuffer, 0, __TX_BUFF_SIZE);

  PHY_Ports[PortNum].TxDatabitLen = 0;
  PHY_Ports[PortNum].TxCount = 0;
}

/**
  * @brief  PRL Hard Reset indication from PHY
  * @param  PortNum  Number of the port
  * @param  val      Appending value to the TX buffer
  * @param  nbit     Number of valid bits
  * @retval None
  */
void PHY_TxBuffer_Append(uint8_t PortNum, uint32_t val, uint8_t nbit)
{
  uint32_t *pBuffer = (uint32_t *)PHY_Ports[PortNum].TxBuffer;
  uint32_t *pOffset = (uint32_t *)&PHY_Ports[PortNum].TxDatabitLen;
  uint32_t map = __BITMASK(nbit);
  uint32_t value = val & map;

  uint32_t pos = *pOffset / __SIZEBIT;
  uint32_t bit = *pOffset % __SIZEBIT;

  /* append the bits */
  pBuffer[pos] |= (value << bit);
  /* checking if there are some bits for next byte */
  if (bit > (__SIZEBIT - MIN(nbit, __SIZEBIT)))
  {
    pBuffer[pos + 1] |= (value >> (__SIZEBIT - bit));
  }

  /* increment the offset according to the appended value */
  *pOffset += nbit;
}

/**
  * @brief  Encode a byte (8bits) to 5B representation (output 10 bit)
  * @param  val    Convert two nibble 8bits in two symbols (10bits)
  * @retval None
  */
__STATIC_INLINE uint32_t PHY_Encode5b_Byte(uint8_t val)
{
  /* using the lookup table, achieve quickly the corresponding 5bit value */
  return coding4b5b[val & 0x0F] | (coding4b5b[(val >> 4) & 0x0F] << 5);
}

/**
  * @brief  Encode a short (16bits) to its 5B representation (output 20 bit)
  * @param  val    Convert four nibble 16bits in four symbols (20bits)
  * @retval None
  */
__STATIC_INLINE uint32_t PHY_Encode5b_Short(uint16_t val)
{
  /* managed calling 2 times the byte conversion */
  return PHY_Encode5b_Byte(val & 0x00FF) | (PHY_Encode5b_Byte((val >> 8) & 0x00FF) << 10);
}

/**
  * @brief  Prepare data packet
  * @param  PortNum      Number of the port
  * @param  Type        PD type
  * @param  pBuffer     Pointer on the PD buffer
  * @param  Size        Size of the PD buffer
  * @retval USBPD Status
  */
USBPD_StatusTypeDef PHY_PreparePacket(uint8_t PortNum, USBPD_SOPType_TypeDef Type, uint8_t *pBuffer, uint8_t Size)
{
  /* get if it is a reset (hard or cable) */
  uint8_t resetRequired = Type == USBPD_SOPTYPE_HARD_RESET || Type == USBPD_SOPTYPE_CABLE_RESET;
  
  /* check the size, according to the following criteria: Exist a n : 2+(n+1)*4 = size, where n is the number of Objects */
  if (!resetRequired  && ((Size < 2) || (Size > 30) || (( (Size - 2) % __SIZE) != 0))) 
  {
    /* the size is not a USBPD PRL message */
    return USBPD_ERROR;
  }

  if (!USBPD_PORT_IsValid(PortNum) || (!resetRequired && pBuffer == NULL))
  {
    return USBPD_ERROR;
  }

  uint32_t value        = 0;
  uint32_t crc_value    = 0;
  uint32_t *pOffset    = (uint32_t *)&PHY_Ports[PortNum].TxDatabitLen;
  uint16_t val16        = 0;
  uint32_t i            = 0;

  /* Clean the Tx buffer */
  PHY_TxBuffer_Reset(PortNum);

  /* Start Of Packet SOP */
  PHY_TxBuffer_Append(PortNum, OrderSets[(uint8_t)Type], OS_BITSIZE);

  if (!resetRequired)
  {
    /* calculate crc */
    crc_value = USBPD_HW_IF_CRC_Calculate(pBuffer, Size);

    /* encoding data */
    for (i = 0; i < Size; i += 2)
    {
      val16 = pBuffer[i] | (pBuffer[i + 1] << 8);
      value = PHY_Encode5b_Short(val16);
      PHY_TxBuffer_Append(PortNum, value, CODE_5B_ITEM4_BITSIZE);
    }

    /* appending CRC (32 bits) */
    value = PHY_Encode5b_Short(crc_value & 0x0000FFFF);
    PHY_TxBuffer_Append(PortNum, value, CODE_5B_ITEM4_BITSIZE);
    value = PHY_Encode5b_Short((crc_value >> 16) & 0x0000FFFF);
    PHY_TxBuffer_Append(PortNum, value, CODE_5B_ITEM4_BITSIZE);

    /* appending EOP */
    PHY_TxBuffer_Append(PortNum, KC_S_EOP, CODE_5B_ITEM1_BITSIZE);
  }

  return FRAME_5B_BitSizeIsValid(*pOffset) ? USBPD_OK : USBPD_ERROR;
}

/**
  * @brief  SOP Detect, according to the spec with not correct bits
  * @param  PortNum  Number of the port
  * @param  OrderSet Ordered Sets
  * @retval SOP Type based on @ref USBPD_SOPType_TypeDef
  */
USBPD_SOPType_TypeDef PHY_SopDetect(uint8_t PortNum, uint32_t OrderSet)
{
  USBPD_SOPType_TypeDef type = USBPD_SOPTYPE_INVALID;

  uint32_t temp;
  uint32_t count, index;

  for (index = 0; index < OS_NUM; index++)
  {
    /* If the orderset mismatchs with one of the available(supported) 
    orderset, the xor result is zero */
    temp = OrderSets[index] ^ OrderSet;
    /* counting number of wrong symbols */
    /* if we find more than 1 wrong symbol pass to next orderset */
    count = 0;
    /* if the orderset is correct temp is zero */
    if (temp)
    {
      /* check if the orderset contains less than 2 error */
      /* check first keycode */
      if (temp & 0x000F8000)
      {
        count++;
      }

      /* check second and if wrong continue */
      if (temp & 0x00007C00)
      {
        count++;
      }
      if (count > 1)
      {
        continue;
      }

      /* check third and if wrong continue */
      if (temp & 0x000003E0)
      {
        count++;
      }
      if (count > 1)
      {
        continue;
      }

      /* check last and if wrong continue */
      if (temp & 0x0000001F)
      {
        count++;
      }
      if (count > 1)
      {
        continue;
      }
    }
#ifdef __STAT
    if (count > 0)
    {
      PHY_Ports[PortNum].Stats.RxSopCorrected++;
    }
#endif
    /* found a valid orderset */
    type = (USBPD_SOPType_TypeDef)index;
    break;
  }

  return type;
}
/**
  * @brief  Decoding 10 bits and return the byte (Decoding 5B/4B)
  * @param  value    Masked to 10 bits
  * @retval return   the decoded value or -1 
  */
__STATIC_INLINE int16_t Decode10Bit(uint32_t value)
{
  uint8_t v0,v1;
  
  v0 = decoding5b4b[(value >>  0) & 0x1F]; /* decoding first nibble */
  v1 = decoding5b4b[(value >>  5) & 0x1F]; /* decoding second nibble */

  /* check if there is an error */
  if (v0 == CODE_5B_INVALID || v1 == CODE_5B_INVALID) 
  {
    return -1;
  }
  
  /* create the decoded value */
  return v0 | (v1<<4);
}

#ifdef __STAT
void PHY_Stat_Reset(uint8_t PortNum)
{
  memset(&PHY_Ports[PortNum].Stats, 0, sizeof(PHY_StatsTypeDef));
}
#endif

/**
 * @brief  Callback to notify the bist is completed
 * @param  PortNum   Number of the port.
 * @param  bistmode  Modality of the bist.
 * @retval none.
 */
void PHY_BistCompleted(uint8_t PortNum, USBPD_BISTMsg_TypeDef bistmode)
{
  if (PHY_Ports[PortNum].cbs->USBPD_PHY_BistCompleted)
  {
    PHY_Ports[PortNum].cbs->USBPD_PHY_BistCompleted(PortNum, bistmode);
  }
}

/**
 * @brief  Callback to notify the a transmission is completed
 * @param  PortNum  Number of the port.
 * @retval none.
 */
void PHY_TxCompleted(uint8_t PortNum)
{
  PHY_Ports[PortNum].State = PHY_StateNone;
  if (PHY_Ports[PortNum].cbs->USBPD_PHY_TxCompleted)
  {
    PHY_Ports[PortNum].cbs->USBPD_PHY_TxCompleted(PortNum);
  }
}

/**
 * @brief  Callback to notify the start of reception
 * @param  PortNum  Number of the port.
 * @retval Status of current reception.
 */
USBPD_PHY_RX_Status_TypeDef PHY_Rx_Reset(uint8_t PortNum)
{
  /* Get the pointer to the decoding structure */
  PHY_RxDecodingTypeDef *pRxData = (PHY_RxDecodingTypeDef *)&PHY_Ports[PortNum].RxDec;

  /* reset the status of the RX process */
  pRxData->Status = USBPD_PHY_RX_STATUS_NONE;

  /* reset the variable of count and memory */
  pRxData->DataCount = 0;
  memset(pRxData->Data, 0, __RX_DATA_LEN);

  /* reset the type of SOP */
  pRxData->MsgType = USBPD_SOPTYPE_INVALID;

  /* reset the type of OrderSet received */
  pRxData->OrderSet = 0x00;

  return pRxData->Status;
}

/**
  * @brief   Callback to notify the new data available for the current reception 
  * @details A state machine to receive data and decode them on fly (during the reception the other bits)
  * @param   PortNum  Number of the port.
  * @param   data     New received data
  * @retval  Status of current reception.
  */
USBPD_PHY_RX_Status_TypeDef PHY_Rx_Accumulate(uint8_t PortNum, uint32_t data) /* 10 bits => 1 byte */
{
  PHY_RxDecodingTypeDef *pRxData = (PHY_RxDecodingTypeDef *)&PHY_Ports[PortNum].RxDec;
  int16_t data4b_temp;
  switch (pRxData->Status)
  {
    /* at start-up the status is none, the preamble is stripped by the low level */
    case USBPD_PHY_RX_STATUS_NONE:
    /* received first part of the SOP, storage it for next check */
      pRxData->OrderSet = data;
      pRxData->Status = USBPD_PHY_RX_STATUS_SOP_DETECTING;
      break;
    case USBPD_PHY_RX_STATUS_SOP_DETECTING:
      /* received second part of the SOP */
      pRxData->OrderSet |= data << 10;

      /* SOP detection */
      pRxData->MsgType = PHY_SopDetect(PortNum, pRxData->OrderSet);
#if 0
      if ((pRxData->MsgType == USBPD_SOPTYPE_SOP) || (pRxData->MsgType == USBPD_SOPTYPE_SOP1) || (pRxData->MsgType == USBPD_SOPTYPE_SOP2))
      {
        pRxData->Status = USBPD_PHY_RX_STATUS_DATA;
      }
      else if (pRxData->MsgType == USBPD_SOPTYPE_HARD_RESET)
      {
        pRxData->Status = USBPD_PHY_RX_STATUS_MESSAGE_READY;
      }
      else if (pRxData->MsgType == USBPD_SOPTYPE_SOP1_DEBUG || \
               pRxData->MsgType == USBPD_SOPTYPE_SOP2_DEBUG || \
               pRxData->MsgType == USBPD_SOPTYPE_CABLE_RESET
              )
      {
        pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP;
      }
      else
      {
        pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP;
      }
#else
      switch (pRxData->MsgType)
      {
        case USBPD_SOPTYPE_HARD_RESET :
          pRxData->Status = USBPD_PHY_RX_STATUS_MESSAGE_READY;
          break;
        case USBPD_SOPTYPE_CABLE_RESET :
          if (PHY_Ports[PortNum].SupportedSOP & 0x1E)
          {
            pRxData->Status = USBPD_PHY_RX_STATUS_MESSAGE_READY;
          }
          else
          {
            pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP;
          }
          break;
        case USBPD_SOPTYPE_SOP :
        case USBPD_SOPTYPE_SOP1 :
        case USBPD_SOPTYPE_SOP2 :
        case USBPD_SOPTYPE_SOP1_DEBUG :
        case USBPD_SOPTYPE_SOP2_DEBUG :
          if (PHY_Ports[PortNum].SupportedSOP & (0x1 << pRxData->MsgType))
          {
            pRxData->Status = USBPD_PHY_RX_STATUS_DATA;
          }
          else
          {
            pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP;
          }
          break;
        default :
          pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP;
          break;
      }
#endif
      break;
    case USBPD_PHY_RX_STATUS_DATA:
      if ((data & CODE_5B_ITEM1_MASK) == KC_S_EOP)
      {
        /* Found a EOP */
        pRxData->Status = USBPD_PHY_RX_STATUS_MESSAGE_READY;
      }
      else
      {
        /* decoding 10 bit and store result */
        data4b_temp = Decode10Bit(data);

        if (data4b_temp == -1)
        {
          pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_INVALID_SYMBOL;
        }
        else
        {
          pRxData->Data[pRxData->DataCount++] = data4b_temp;
        }
      }
      //the status don't change
      break;
    default:
      //RxStatus = USBPD_PHY_RX_STATUS_ERROR;
      break;
  }

  return pRxData->Status;
}

/**
  * @brief  Callback to notify the end of the current reception
  * @param  PortNum  Number of the port
  * @retval Status of current reception
  */
USBPD_PHY_RX_Status_TypeDef PHY_Rx_Completed(uint8_t PortNum)
{
  uint32_t crc_read, crc_calc;
  PHY_RxDecodingTypeDef *pRxData = (PHY_RxDecodingTypeDef *)&PHY_Ports[PortNum].RxDec;
  if (pRxData->Status != USBPD_PHY_RX_STATUS_MESSAGE_READY)
  {
#ifdef __STAT
    PHY_Ports[PortNum].Stats.RX2Error++;

    switch (pRxData->Status)
    {
      case USBPD_PHY_RX_STATUS_ERROR_UNSUPPORTED_SOP:
        PHY_Ports[PortNum].Stats.RX2ErrorUnsupportedSOP++;
        break;
      case USBPD_PHY_RX_STATUS_ERROR_INVALID_SOP:
        PHY_Ports[PortNum].Stats.RX2ErrorInvalidSOP++;
        break;
      case USBPD_PHY_RX_STATUS_ERROR_INVALID_SYMBOL:
        PHY_Ports[PortNum].Stats.RX2ErrorInvalidSymbol++;
        break;
      case USBPD_PHY_RX_STATUS_ERROR_EOP_NOT_FOUND:
        PHY_Ports[PortNum].Stats.RX2ErrorEOP++;
        break;
      default :
        break;
    }
#endif

    return pRxData->Status;
  }

  if (pRxData->MsgType == USBPD_SOPTYPE_HARD_RESET || pRxData->MsgType == USBPD_SOPTYPE_CABLE_RESET)
  {
    /* received a hard reset, call the callback */
#ifdef __STAT
    PHY_Ports[PortNum].Stats.RX2Special++;
#endif
    if (PHY_Ports[PortNum].cbs->USBPD_PHY_ResetIndication != NULL)
    {
      PHY_Ports[PortNum].cbs->USBPD_PHY_ResetIndication(PortNum, pRxData->MsgType);
    }
    return pRxData->Status = USBPD_PHY_RX_STATUS_OK;
  }

  /* incoming message is a valid SOP with an EOP */
  /* set the pointer to the RxBuffer from PRL */
  uint8_t *pRxBuffer = PHY_Ports[PortNum].pRxBuffer;

  /* calculate the crc for the incoming message */
  crc_calc = USBPD_HW_IF_CRC_Calculate(pRxData->Data, pRxData->DataCount - 4);

  /* get the crc from the incoming message */
  uint8_t *pCrcField = &pRxData->Data[pRxData->DataCount - 4];
  crc_read = *pCrcField;
  pCrcField++;
  crc_read |= ((*pCrcField) << 8);
  pCrcField++;
  crc_read |= ((*pCrcField) << 16);
  pCrcField++;
  crc_read |= ((*pCrcField) << 24);
  pCrcField++;

  /* if it isn't ok discard the incoming message */
  if (crc_read != crc_calc)
  {
#ifdef __STAT
    PHY_Ports[PortNum].Stats.RX2Error++;
    PHY_Ports[PortNum].Stats.RX2ErrorCRC++;
#endif
    return pRxData->Status = USBPD_PHY_RX_STATUS_ERROR_CRC_FAILED; //AR
  }

  /* copy the data to rxbuffer */
  memcpy(pRxBuffer, pRxData->Data, pRxData->DataCount - 4);

  /* call the callback */
#ifdef __STAT
  PHY_Ports[PortNum].Stats.RX2Ok++;
#endif
  if (PHY_Ports[PortNum].cbs->USBPD_PHY_MessageReceived != NULL)
  {
    PHY_Ports[PortNum].cbs->USBPD_PHY_MessageReceived(PortNum, pRxData->MsgType);
  }

  return pRxData->Status = USBPD_PHY_RX_STATUS_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

