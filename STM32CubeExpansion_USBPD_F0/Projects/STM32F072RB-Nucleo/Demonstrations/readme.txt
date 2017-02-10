/**
  @page Demo   USB-C Power Delivery Demonstration Image
 
  @verbatim
  ******************** (C) COPYRIGHT 2016 STMicroelectronics *******************
  * @file    Provider/readme.txt 
  * @author  MCD Application Team
  * @version V1.2.0
  * @date    17-Jan-2017
  * @brief   Description of the USB-C Power Delivery Demonstration.
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. All rights reserved.
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
  @endverbatim

@par Demo Description

The STM32F0 X-CUBE-USB-PD demonstration comes on top of the STM32 USB-PD library
to show a usage case with 2 ports.

For a detailed description of how to try this demo and its features, please refer
to following documents available on www.st.com :
UM2051: Getting started with the STM32 Nucleo pack for USB Type-C™ and Power Delivery
UM2050: STM32 Nucleo pack for USB Type-C™ and Power Delivery with the Nucleo-F072RB board

@par Demonstration Overview

Both PORT_0 and PORT_1 are configured as DRP (Provider/Consumer).
Ports will switch from one role to the other each four seconds. The role will be
highlighted by the blue LED (LED D203 for Port 0 and LED D200 for Port 1) :
- when attached blinking once or twice for respectively Provider or Consumer role.
- when not attached, blinking three times indicating DRP support.
Once the two Type-C ports of the expansion board are connected together by the Type-C to
Type-C cable, the demonstration shows the cable attachment/detachment operation and
orientation mechanism by LEDs blinking.
After the procedure described in "Configuration setup" section in UM2051, the following 
actions must be accomplished to run the embedded demonstration:

 1. Connect the two Type-C receptacles on the expansion board using the USB Type-C
    cable (provided).
 2. Blue LED (D203, resp. D200) will blink once when the PORT_0 (resp. PORT_1) is 
    working as Provider, while it will blink twice when the port is working as Consumer.
 3. The two orange LEDs (D205 and D202) will blink once or twice indicating the CC line
    used respectively for PORT_0 and PORT_1. Changing the cable insertion, the LEDs
    will blink according to the cable orientation.
 4. Green LEDs (D201 and D204) will turn on when the port, working as Provider, is
    supplying the Port Partner or, working as Consumer, is sinking power
    (Explicit Contract).

@par Hardware and Software environment

  - This application runs on STM32F072RB devices.
  
  - This demo has been developed and tested with STMicroelectronics STM32F072RB-Nucleo 
    RevC board with P-NUCLEO-USB001 shield RevB connected on CN7 and CN10 connectors.

  - Connect the STM32 Nucleo board to the USB-C Power Delivery consumer through
    USB typeC cable to the connector CN0 in the X-NUCLEO shield.
	To test this application, the Consumer_RTOS application can be used as a 
	consumer.

  - STM32F072RB-Nucleo RevC Set-up
    Please refer to "Configuration setup" section in UM2051. 

@par How to use it ? 

Below the detailed steps:

In order to make the program work, you must do the following :
 1 - Open STM32 ST-Link Utility
 2 - Connect the STM32F072RB-Nucleo board to the host using STLink USB port.
 3 - Use "PNUCLEO-USB001-DEMO1.bin" file with STM32 ST-Link Utility to program the device's internal Flash.
 4 - Reset the board using Reset button.
 5 - Run the demonstration

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 
