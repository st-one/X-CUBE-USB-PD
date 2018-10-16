/**
  ******************************************************************************
  * @file    bmc.c
  * @author  System Lab
  * @brief   BMC functions
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
#include "bmc.h"
#include <stdio.h>

/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
  * @{
  */

/** @addtogroup USBPD_DEVICE_BMC
  * @{
  */

static const uint8_t BMC_Coding[2][16] =
{
  {
    /* Previous 0 */
    /* 0 0000 */  0x33 /* 00110011 */,
    /* 1 0001 */  0xCD /* 11001101 */,
    /* 2 0010 */  0xCB /* 11001011 */,
    /* 3 0011 */  0x35 /* 00110101 */,
    /* 4 0100 */  0xD3 /* 11010011 */,
    /* 5 0101 */  0x2D /* 00101101 */,
    /* 6 0110 */  0x2B /* 00101011 */,
    /* 7 0111 */  0xD5 /* 11010101 */,
    /* 8 1000 */  0xB3 /* 10110011 */,
    /* 9 1001 */  0x4D /* 01001101 */,
    /* A 1010 */  0x4B /* 01001011 */,
    /* B 1011 */  0xB5 /* 10110101 */,
    /* C 1100 */  0x53 /* 01010011 */,
    /* D 1101 */  0xAD /* 10101101 */,
    /* E 1110 */  0xAB /* 10101011 */,
    /* F 1111 */  0x55 /* 01010101 */,
  },
  {
    /* Previous 1 */
    /* 0 0000 */  0xCC /* 11001100 */,
    /* 1 0001 */  0x32 /* 00110010 */,
    /* 2 0010 */  0x34 /* 00110100 */,
    /* 3 0011 */  0xCA /* 11001010 */,
    /* 4 0100 */  0x2C /* 00101100 */,
    /* 5 0101 */  0xD2 /* 11010010 */,
    /* 6 0110 */  0xD4 /* 11010100 */,
    /* 7 0111 */  0x2A /* 00101010 */,
    /* 8 1000 */  0x4C /* 01001100 */,
    /* 9 1001 */  0xB2 /* 10110010 */,
    /* A 1010 */  0xB4 /* 10110100 */,
    /* B 1011 */  0x4A /* 01001010 */,
    /* C 1100 */  0xAC /* 10101100 */,
    /* D 1101 */  0x52 /* 01010010 */,
    /* E 1110 */  0x54 /* 01010100 */,
    /* F 1111 */  0xAA /* 10101010 */,
  }
};

#define TOGGLE_BIT_N(__VAL__,__N__)         (( (__VAL__) >> (__N__) ) & 0x0001 )

static inline uint16_t BMC_Coding_Byte(uint8_t Val, uint16_t PreviousVal)
{
  uint16_t res = 0;
  uint8_t *p = (uint8_t *)&res;
  p[0] = BMC_Coding[(TOGGLE_BIT_N(PreviousVal, 15)) ][Val & 0x0F];
  p[1] = BMC_Coding[ TOGGLE_BIT_N(p[0], 7) ][(Val >> 4) & 0x0F];
  return res;
}

USBPD_StatusTypeDef BMC_MakeCoding(
  uint8_t *pSourceBuffer,
  uint16_t SourceBitsize ,
  uint16_t *pDestBuffer,
  uint16_t *pDestBitsize,
  uint16_t MaxDestSize, /* bytes */
  uint16_t StartupValue,
  uint8_t *pLastBit)
{
  if (pSourceBuffer == NULL || pDestBuffer == NULL || SourceBitsize == 0)
  {
    return USBPD_ERROR;
  }

  uint16_t len = DIV_ROUND_UP(SourceBitsize, 8);
  if (len > MaxDestSize / 2)
  {
    return USBPD_ERROR;
  }

  uint16_t sLastItemBitOffset = (SourceBitsize % 8) << 1;
  uint16_t sLastItemMap;
  if (sLastItemBitOffset)
  {
    sLastItemMap = (1 << sLastItemBitOffset) - 1; //create the map of the last short
  }
  else
  {
    sLastItemBitOffset = 16;
    sLastItemMap = 0xFFFF;
  }

  uint16_t *pLastItem = NULL;
  int count = 1;

  pDestBuffer[0] = BMC_Coding_Byte(pSourceBuffer[0], StartupValue);
  for (count = 1; count < len; count++)
  {
    pDestBuffer[count] = BMC_Coding_Byte(pSourceBuffer[count], pDestBuffer[count - 1]);
  }

  pLastItem = &pDestBuffer[len - 1];

  if (pDestBitsize != NULL)
  {
    *pDestBitsize = SourceBitsize * 2;
  }
  *pLastItem &= sLastItemMap; //reset bit not used in the last byte
  if (pLastBit != NULL)
  {
    *pLastBit = (*pLastItem >> (sLastItemBitOffset - 1)) & 0x01;
  }

  return USBPD_OK;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

