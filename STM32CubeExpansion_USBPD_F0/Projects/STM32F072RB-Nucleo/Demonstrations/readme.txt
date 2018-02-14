/**
  @page USB-C Power Delivery Demonstrations Images for P-NUCLEO-USB002/ P-NUCLEO-USB001 kits

  @verbatim
  ******************** (C) COPYRIGHT 2017 STMicroelectronics *******************
  * @file    Demonstrations/readme.txt 
  * @author  MCD Application Team
  * @brief   Description of the USB-C Power Delivery Demonstrations.
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. All rights reserved.
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

  
Two demonstrations are available.
Demonstration for PNUCLEO-USB002-DEMO1 is below PNUCLEO-USB001-DEMO1 details.
  
@par PNUCLEO-USB001-DEMO1 Description

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


@par : second demonstration binary.

The STM32F0 X-CUBE-USB-PD demonstration also comes on top of the STM32 USB-PD library
to show a usage case with 2 ports, in case of use with  STM32F072+STUSB1602 devices.

For a detailed description of how to try this demo and its features, please refer
to following documents available on www.st.com :
UM2191: STM32 Nucleo pack for USB Type-C and Power Delivery with the Nucleo-F072RB board and the STUSB1602
UM2205: Getting started with the STM32 Nucleo pack for USB Type-C and Power Delivery with the Nucleo-F072RB and STUSB1602
DB3148: USB Type-C and Power Delivery Nucleo pack with NUCLEO-F072RB expansion board based on STUSB1602 


@par PNUCLEO-USB002-DEMO1 Overview

At startup:
 - Status LED (Blue LED D106) is on.
 - Role LEDs (Blue LEDs D100, D103) blink three times meaning that the board is acting the DRP role.
 - CC LEDs (Orange LEDs D102, D105) are off.
 - VBUS LEDs (Green LEDs D101, D104) are off.

Because of the P-NUCLEO-USB002 acts the DRP role, the user can connect to port 0 or port 1 receptacle an application 
which plays the consumer role as well as the provider role.
When a consumer is connected:
 - Role LED will blink one time to show that the device swiched to Provider.
 - CC LED will blink once if connected on CC1, twice if connected on CC2.
 - The STM32 MCU behaves as a Provider (Source mode), it exchanges Power profiles with 
   the connected device and waits for Power Request message from the attached consumer.
   While the communication is carried on, VBUS LED will blink.
   If the requested power can be met, the communication will end with an Accept message followed 
   by a PS_RDY message.
   Once the Explicit Contract is achieved, VBUS LED is on to indicate that the Power Contract was established.
When a provider is connected:
 - Role LED will blink twice to show that the device swiched to Consumer.
 - CC LED will blink once if connected on CC1, twice if connected on CC2.
 - The STM32 MCU behaves as a Consumer (Sink mode),it waits for Power Capabilities message 
   from the attached provider. When a Source Capabilities message is received, the STM32 
   starts the evaluation of the received capabilities and check if one of the received power 
   objects can meet its power requirement.
   While the communication is carried on, VBUS LED will blink.
   The STM32 shall send the Request message to request the new power level from the offered 
   Source Capabilities.
   Once the Explicit Contract is achieved (PS_Ready message received), VBUS LED VBUS LED is on to 
   indicate that the Power Contract was established.

The power role swap is also supported:
 - When connected to an USB-C provider only device (source mode), the power role swap between the two boards is not possible,
   DRP board will act automatically as a sink.
 - When connected to an USB-C consumer only device (sink mode), the power role swap between the two boards is not possible,
   DRP board will act automatically as a source.
 - When connected to an USB-C with DRP, the power role swap could be triggered by CLI command.

The demo application can be managed with two different supply options according to the fact that the board is supplied by
the on-board STM32F072RB-Nucleo RevC voltage regulator by mean of a USB Type-A to Mini-B cable plugged to the CN1 connector and then to a PC
as well as it is supplied by mean of the VBUS delivered by the Provider attached or The P-NUCLEO-USB002 is connected to an external
power board by the connector CN4.

if the demo application is supplied by the on-board STM32F072RB-Nucleo RevC voltage regulator, by mean of a USB Type-A to Mini-B cable
plugged to the CN1 connector and then to a PC:
  - On STM32F072RB-Nucleo RevC board, verify that the jumper JP1 is open, jumper JP5 (PWR) is closed on U5V 
  (fitting the pins 1-2), and jumper JP6 (IDD) is closed.

On the expansion board:
  - jumpers JP400 and JP401 are closed.
  - jumpers JP000 and JP001 are closed on pin 1-2.


@par Hardware and Software environment

  - This application runs on STM32F072RB devices.
  
  - This demonstration has been implemented with P-NUCLEO-USB002 kit made by STMicroelectronics STM32F072RB-Nucleo RevC
    board and the expansion board RevB connected on top of CN7 and CN10 connectors. 
    The provided example can be easily tailored to any other supported device and development board.

  - Connect the P-NUCLEO-USB002 kit to the a USB-C Power Delivery consumer, provider or DRP attaching the
    USB typeC cable to the receptacle CN0 /CN1 on the expansion board.
    To test this application, another P-NUCLEO-USB002 kit running the Consumer_RTOS, Provider_RTOS or DRP_RTOS role can be used.

  - STM32F072RB-Nucleo RevC Set-up
      - SB48, SB49, SB62 and SB63 must be closed
      - SB13, SB14, SB15 and SB21 must be removed
      - R34 and R36 must be removed
      - JP5 Jumper must be connected to U5V.

  - The Demo application is augmented with a CLI shell.
    To use it, verify that CN2 pins of the expansion board are conneced to CN3 pins of STM32F072RB-Nucleo board crossing TX and RX signals. 
    Use a shell configuring serial communication according to the following parameters:
	- Speed (baud rate): 115200
	- Data bits: 8
	- Stop bits: 1
	- Parity: none
    After the communication is established, type '?' and press <enter> to access the menu showing the available commands.


@par How to use it ?

 1 - Open STM32 ST-Link Utility
 2 - Connect the STM32F072RB-Nucleo board to the host using STLink USB port.
 3 - Use "PNUCLEO-USB002-DEMO1.bin" file with STM32 ST-Link Utility to program the device's internal Flash.
 4 - Reset the board using Reset button.
 5 - Run the demonstration

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
 
