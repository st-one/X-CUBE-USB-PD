/**
  ******************************************************************************
  * @file    usbpd_pe.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    22-June-2016
  * @brief   Header file of Policy Engine module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics International N.V. 
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
#ifndef __USBPD_PE_H
#define __USBPD_PE_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "usbpd_def.h"
#include "string.h"

/* Exported define -----------------------------------------------------------*/
#define USBPD_MAX_NB_PDO                 ((uint32_t)7)                /*!< Maximum number of supported Power Data Objects: fix by the Specification */
#define USBPD_MAX_RX_BUFFER_SIZE         ((uint32_t)30)               /*!< Maximum size of Rx buffer                                                */
#define USBPD_MAX_TX_BUFFER_SIZE         ((uint32_t)30)               /*!< Maximum size of Tx buffer                                                */
#define PE_TIMER_ENABLE_MSK              ((uint32_t)0x80000000)       /*!< Enable Timer Mask                                                        */
#define PE_TIMER_READ_MSK                ((uint32_t)0x7FFFFFFF)       /*!< Read Timer Mask                                                          */

/* Policy Engine Timers */
#define PE_TIMEOUT_VALUE                 ((uint32_t)6000)      /*!< 6000 ms     */

/* Policy Engine Timers */
#define PE_TNORESPONSE_MIN               ((uint32_t)4500)      /*!< 4500 ms     */
#define PE_TNORESPONSE_MAX               ((uint32_t)5500)      /*!< 5500 ms     */
#define PE_TPSHARDRESET_MIN              ((uint32_t)30)        /*!< 30 ms       */
#define PE_TPSHARDRESET_MAX              ((uint32_t)35)        /*!< 35 ms       */
#define PE_TSRCRECOVERHARDRESET_MIN      ((uint32_t)800)       /*!< 660 ms     */
#define PE_TSRCRECOVERHARDRESET_MAX      ((uint32_t)1000)      /*!< 1000 ms     */
#define PE_TPSSRCOFF_MIN                 ((uint32_t)750)       /*!< 750 ms      */
#define PE_TPSSRCOFF_MAX                 ((uint32_t)920)       /*!< 920 ms      */
#define PE_TPSSRCON_MIN                  ((uint32_t)390)       /*!< 390 ms      */
#define PE_TPSSRCON_MAX                  ((uint32_t)480)       /*!< 480 ms      */
#define PE_TPSTRANSITION_MIN             ((uint32_t)450)       /*!< 450 ms      */
#define PE_TPSTRANSITION_MAX             ((uint32_t)550)       /*!< 550 ms      */
#define PE_TSENDERRESPONSE_MIN           ((uint32_t)24)        /*!< 24 ms       */
#define PE_TSENDERRESPONSE_MAX           ((uint32_t)30)        /*!< 30 ms       */
#define PE_TSNKACTIVITY_MIN              ((uint32_t)120)       /*!< 120 ms      */
#define PE_TSNKACTIVITY_MAX              ((uint32_t)150)       /*!< 150 ms      */
#define PE_TSNKREQUEST                   ((uint32_t)100)       /*!< 100 ms      */

#define PE_TSNKWAITCAP_MIN               ((uint32_t)310)       /*!< 310 ms      */
#define PE_TSNKWAITCAP_MAX               ((uint32_t)620)       /*!< 620 ms      */

#define PE_TSRCACTIVITY_MIN              ((uint32_t)40)        /*!< 40 ms       */
#define PE_TSRCACTIVITY_MAX              ((uint32_t)50)        /*!< 50 ms       */

#define PE_TSENDSRCCAP_MIN               ((uint32_t)100)       /*!< 100 ms      */
#define PE_TSENDSRCCAP_MAX               ((uint32_t)200)       /*!< 200 ms      */

#define PE_TSRCTRANSITION_MIN            ((uint32_t)25)        /*!< 25 ms       */
#define PE_TSRCTRANSITION_MAX            ((uint32_t)35)        /*!< 35 ms       */
#define PE_TSWAPRECOVER_MIN              ((uint32_t)500)       /*!< 500 ms      */
#define PE_TSWAPRECOVER_MAX              ((uint32_t)1000)      /*!< 1000 ms     */
#define PE_TSWAPSRCSTART_MIN             ((uint32_t)20)        /*!< 20 ms       */

#define PE_TBISTCONTMODE_MIN             ((uint32_t)30)        /*!< 30 ms       */
#define PE_TBISTCONTMODE_MAX             ((uint32_t)60)        /*!< 60 ms       */

/* Policy Engine Counters */
#define PE_NMESSAGEIDCOUNT               ((uint32_t)7)         /*!< Detect duplicate Messages                                       */
#define PE_NRETRYCOUNT                   ((uint32_t)3)         /*!< Retry Counter when Message transmission failure                 */
#define PE_NHARDRESETCOUNT               ((uint32_t)2)         /*!< Retry Counter when there is no response from the remote device  */
#define PE_NCAPSCOUNT                    ((uint32_t)50)        /*!< The number of Source_Capabilities Messages sent                 */

/* PDO : Power Data Object:
 * 1. The vSafe5V Fixed Supply Object shall always be the first object.
 * 2. The remaining Fixed Supply Objects,
 *    if present, shall be sent in voltage order; lowest to highest.
 * 3. The Battery Supply Objects,
 *    if present shall be sent in Minimum Voltage order; lowest to highest.
 * 4. The Variable Supply (non battery) Objects,
 *    if present, shall be sent in Minimum Voltage order; lowest to highest.
 */
#define PE_PDO_TYPE_FIXED                ((uint32_t)0x00000000)      /*!< Fixed Supply PDO mask          */
#define PE_PDO_TYPE_BATTERY              ((uint32_t)0x80000000)      /*!< Battery Supply PDO mask        */
#define PE_PDO_TYPE_VARIABLE             ((uint32_t)0x40000000)      /*!< Variable Supply PDO mask       */
#define PE_PDO_TYPE_MASK                 ((uint32_t)0xC0000000)      /*!< PDO type mask                  */

/* Exported typedef ----------------------------------------------------------*/
/** @defgroup PE_CallBacks_structure_definition PE CallBacks structure definition
  * @brief  PE CallBacks exposed by the PE to the  DMP
  * @{
  */
typedef struct
{
  /**
    * @brief  Request the DPM to setup the new power level.
    * @param  hport: The current port number
    * @param  rdoposition: RDO position in the table of PDO (possible value from 1 to 7)
    * @retval None
  */
  void (*USBPD_PE_RequestSetupNewPower)(uint8_t hport, uint8_t rdoposition);
  
  /**
    * @brief  Request the DPM to perform a HardReset.
    * @param  hport: The current port number
    * @retval None
  */
  void (*USBPD_PE_HardReset)(uint8_t hport);

  /**
    * @brief  Get evaluation of swap request from DPM.
    * @param  hport: The current port number
    * @retval USBPD_OK if PR_swap is possible, else USBPD_ERROR
  */
  USBPD_StatusTypeDef (*USBPD_PE_EvaluatPRSwap)(uint8_t hport);

  /**
    * @brief  Request the DPM to turn Off power supply.
    * @param  hport: The current port number
    * @retval None
  */
  void (*USBPD_PE_TurnOffPower)(uint8_t hport);

  /**
    * @brief  Request the DPM to turn On power supply.
    * @param  hport: The current port number
    * @retval None
  */
  void (*USBPD_PE_TurnOnPower)(uint8_t hport);
  
  /**
    * @brief  Request the DPM to assert Rd.
    * @param  hport: The current port number
    * @retval None
  */
  void (*USBPD_PE_AssertRd)(uint8_t hport);

  /**
    * @brief  Request the DPM to assert Rp.
    * @param  hport: The current port number
    * @retval None
  */
  void (*USBPD_PE_AssertRp)(uint8_t hport);
  
  /**
    * @brief  Inform DPM that an Explicit contract is established.
    * @param  hport: The current port number
    * @retval None
  */
  void (*USBPD_PE_ExplicitContractDone)(uint8_t hport);  
}USBPD_PE_Callbacks;


/** @defgroup PE_Source_state_structure_definition PE Source state structure definition
  * @brief  PE Source State structure definition
  * @{
  */
typedef enum
{
  PE_SRC_HARD_RESET_RECEIVED              = 0x00,  /*!< Hard reset signaling is detected                                  */
  PE_SRC_STARTUP                          = 0x01,  /*!< PE Source starting state                                          */
  PE_SRC_TRANSITION_TO_DEFAULT            = 0x02,  /*!< Return to default state                                           */
  PE_SRC_DISCOVERY                        = 0x03,  /*!< PE Source in discovery state                                      */
  PE_SRC_DISABLED                         = 0x04,  /*!< PE Source disabled                                                */
  PE_SRC_HARD_RESET                       = 0x05,  /*!< Request a hard reset                                              */
  PE_SRC_SEND_CAPABILITIES                = 0x06,  /*!< Send a Source_Capabilities Message                                */
  PE_SRC_WAIT_NEW_CAPABILITIES            = 0x07,  /*!< Waiting for a new Capabilities from the DPM                       */
  PE_SRC_NEGOTIATE_CAPABILITY             = 0x08,  /*!< Evaluate the Request from the attached Sink                       */
  PE_SRC_CAPABILITY_RESPONSE              = 0x09,  /*!< Request received from the Sink                                    */
  PE_SRC_TRANSITION_SUPPLY                = 0x0A,  /*!< Change power supply to another Source                             */
  PE_SRC_READY                            = 0x0B,  /*!< Power supply is ready                                             */
  PE_SRC_GIVE_SRC_CAP                     = 0x0C,  /*!< Send a Source_Capabilities Message                                */
  PE_SRC_GET_SNK_CAP                      = 0x0D,  /*!< Request the capabilities from the attached Sink                   */
  PE_SRC_SOFT_RESET_RECEIVED              = 0x0E,  /*!< Soft reset message received: Reset the PRL then 
                                                        request the PRL Layer to send an Accept Message                   */
  PE_SRC_SEND_SOFT_RESET                  = 0x0F,  /*!< Request the PRL Layer to perform a Soft Reset 
                                                        then send a Soft_Reset Message to the Sink                        */
  PE_SRC_PING                             = 0x10,  /*!< Send Ping message                                                 */
  PE_SRC_IDLE                             = 0x11,  /*!< Idle state                                                        */
  PE_SRC_EXPLICIT_CONTRACT                = 0x12,  /*!< Explicit Contract state                                           */
  
  /* Source to Sink Power Role Swap state definition */
  PE_PRS_SRC_SNK_EVALUATE_SWAP            = 0x13,  /*!< Evaluate PR_Swap from Source to Sink                              */
  PE_PRS_SRC_SNK_ACCEPT_SWAP              = 0x14,  /*!< Send Accept message in response to PR_Swap command                */
  PE_PRS_SRC_SNK_TRANSITION_TO_OFF        = 0x15,  /*!< Wait tSrcTransition and request the DPM to turn off power supply  */
  PE_PRS_SRC_SNK_ASSERT_RD                = 0x16,  /*!< Request DPM to assert Rd                                          */
  PE_PRS_SRC_SNK_WAIT_SOURCE_ON           = 0x17,  /*!< Send PS_RDY message                                               */
  PE_PRS_SRC_SNK_SEND_SWAP                = 0x18,  /*!< Send PR_Swap message                                              */
  PE_PRS_SRC_SNK_REJECT_SWAP              = 0x19,  /*!< Send Reject message                                               */
   
  /* BIST Carrier Mode 2 and test data */
  PE_SRC_BIST_CARRIER_MODE_2              = 0x20,  /*!< BIST Carrier Mode 2                                               */
  PE_SRC_BIST_TEST_DATA                   = 0x21,  /*!< BIST Test Data                                                    */
  
  PE_SRC_SEND_REJECT_MSG                  = 0x22   /*!< Send Reject message if command is not supported                   */
}PE_SRCState_TypeDef;
/** 
  * @}
  */


/** @defgroup PE_Sink_state_structure_definition PE Sink state structure definition
  * @brief  PE Sink State structure definition  
  * @{
  */
typedef enum
{
  PE_SNK_HARD_RESET_RECEIVED              = 0x00,  /*!< Hard reset signaling received                                              */
  PE_SNK_STARTUP                          = 0x01,  /*!< PE Sink starting state                                                     */
  PE_SNK_TRANSITION_TO_DEFAULT            = 0x03,  /*!< Return to default state                                                    */
  PE_SNK_DISCOVERY                        = 0x04,  /*!< Waiting for VBUS to be present                                             */
  PE_SNK_HARD_RESET                       = 0x05,  /*!< Request a hard reset                                                       */
  PE_SNK_WAIT_FOR_CAPABILITIES            = 0x06,  /*!< Waiting for a capabilities message from the Source                         */
  PE_SNK_EVALUATE_CAPABILITY              = 0x07,  /*!< Request the DPM to evaluate the supplied Source capabilities               */
  PE_SNK_SELECT_CAPABILITY                = 0x08,  /*!< Send a response message based on the capabilities evaluation from the DPM  */
  PE_SNK_TRANSITION_SNK                   = 0x09,  /*!< Change the power supply to the new power level                             */
  PE_SNK_READY                            = 0x0A,  /*!< Sink operating at a stable power level                                     */
  PE_SNK_GIVE_SNK_CAP                     = 0x0B,  /*!< Send a Sink_Capabilities message                                           */
  PE_SNK_GET_SRC_CAP                      = 0x0C,  /*!< Send a Get_Source_Cap message                                              */
  PE_SNK_SOFT_RESET_RECEIVED              = 0x0D,  /*!< Soft reset message received: Reset the PRL then request the PRL Layer to 
                                                        send an Accept Message                                                     */
  PE_SNK_SEND_SOFT_RESET                  = 0x0E,  /*!< Request the PRL Layer to perform a Soft Reset then send a Soft_Reset 
                                                        Message to the Source                                                      */
  PE_SNK_IDLE                             = 0x0F,  /*!< Idle state                                                                 */
  PE_SNK_WAIT_RECEIVED                    = 0x10,  /*!< Wait command received                                                      */
  PE_SNK_EXPLICIT_CONTRACT                = 0x11,  /*!< Explicit Contract state                                                    */
  
   /* Sink to Source Power Role Swap state definition */
  PE_PRS_SNK_SRC_EVALUATE_SWAP            = 0x12,  /*!< Evaluate PR_Swap from Sink to Source                                       */
  PE_PRS_SNK_SRC_ACCEPT_SWAP              = 0x13,  /*!< Send Accept message in response to PR_Swap command                         */
  PE_PRS_SNK_SRC_TRANSITION_TO_OFF        = 0x14,  /*!< Wait tSrcTransition and request the DPM to turn off power Sink             */
  PE_PRS_SNK_SRC_ASSERT_RP                = 0x15,  /*!< Request DPM to assert Rp                                                   */
  PE_PRS_SNK_SRC_SOURCE_ON                = 0x16,  /*!< Send PS_RDY message                                                        */
  PE_PRS_SNK_SRC_SEND_SWAP                = 0x17,  /*!< Send PR_Swap message                                                       */
  PE_PRS_SNK_SRC_REJECT_SWAP              = 0x18,  /*!< Send Reject message                                                        */
  
  /* BIST Carrier Mode 2 and test data */
  PE_SNK_BIST_CARRIER_MODE_2              = 0x19,  /*!< BIST Carrier Mode 2                                                        */
  PE_SNK_BIST_TEST_DATA                   = 0x20,  /*!< BIST Test Data                                                             */
  
  PE_SNK_SEND_REJECT_MSG                  = 0x21   /*!< Send Reject message if command is not supported                            */
}PE_SNKState_TypeDef;
/** 
  * @}
  */


/** @defgroup USBPD_PETimers_TypeDef PE Timers Structure definition
  * @brief  PE Timers Structure definition
  * @{
  */
typedef struct
{
  /* Policy Engine variables */
  __IO uint32_t PE_PSHardResettSrcRecover;        /*!< tSrcRecover timer                                          */
  __IO uint32_t PE_PSHardResetTimer;              /*!< PSHardResetTimer timer, the valuue can be tPSHardReset     */
  __IO uint32_t PE_PSSRCOffTimer;                 /*!< PSSourceOffTimer timer, the value can tPSSrcOff            */
  __IO uint32_t PE_PSSRCOnTimer;                  /*!< PSSourceOnTimer timer, the value can tPSSrcOn              */
  __IO uint32_t PE_SenderResponseTimer;           /*!< SenderResponseTimer timer, the value can tSenderResponse   */
  __IO uint32_t PE_SwapRecoveryTimer;             /*!< SwapRecoveryTimer timer, the value can tSwapRecover        */
  
  /* Source Policy Engine variables */
  __IO uint32_t PE_SRCNoResponseTimer;            /*!< SRC NoResponseTimer timer, the value can tNoResponse       */
  __IO uint32_t PE_SRCActivityTimer;              /*!< SourceActivityTimer timer, the value can tSrcActivity      */
  __IO uint32_t PE_SRCCapabilityTimer;            /*!< SourceCapabilityTimer timer, the value can tSrcActivity    */
  __IO uint32_t PE_tSrcTransition;                /*!< tSrcTransition timer                                       */
  __IO uint32_t PE_SwapSrcStartTimer;             /*!< tSwapSourceStart timer, the value can tSwapSourceStart     */
  
  /* Sink Policy Engine variables */ 
  __IO uint32_t PE_SNKNoResponseTimer;            /*!< SNK NoResponseTimer timer, the value can tNoResponse       */
  __IO uint32_t PE_SNKPSTransitionTimer;          /*!< PSTransitionTimer timer, the value can tPSTransition       */
  __IO uint32_t PE_SNKActivityTimer;              /*!< SinkActivityTimer timer, the value can tSnkActivity        */
  __IO uint32_t PE_SNKRequestTimer;               /*!< SinkRequestTimer timer, the value can tSnkRequest          */
  __IO uint32_t PE_SNKWaitCapTimer;               /*!< SinkWaitCapTimer timer, the value can tSnkWaitCap          */
  
  /* BIST Timer */
  __IO uint32_t PE_BISTContModeTimer;             /*!< BISTContModeTimer timer, the value can tBISTContMode       */
  __IO uint32_t PE_BISTStartTimer;                /*!< BISTStartTimer timer, the value defined by Tester          */
}USBPD_PETimers_TypeDef;
/** 
  * @}
  */


/** @defgroup USBPD_PECounters_TypeDef PE Counters Structure definition
  * @brief  PE Counters Structure definition
  * @{
  */
typedef struct
{
  __IO uint32_t PE_HardResetCounter;              /*!< Hard Reset Counter, the value can be nHardResetCount                               */
  __IO uint32_t PE_CapsCounter;                   /*!< Source_Capabilities Messages CapsCounter Counter, the value can be nCapsCount      */
}USBPD_PECounters_TypeDef;
/** 
  * @}
  */


/** @defgroup USBPD_SNKFixedRaquest_TypeDef PE Sink requested power profile Structure definition
  * @brief  PE Sink requested power profile Structure definition
  * @{
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
/** 
  * @}
  */


/**
  * @brief  USB Power Delivery command states definition
  */
typedef enum 
{
  USBPD_CMD_IDLE  = 0x00,
  USBPD_CMD_SEND  = 0x01,
  USBPD_CMD_WAIT  = 0x02
} USBPD_CMD_StateTypeDef; 


/** @defgroup USBPD_CablePlug_TypeDef USB PD Cable Plug Types structure definition
  * @brief  USB PD Cable Plug Types structure definition
  * @{
  */
typedef enum
{
  USBPD_CABLEPLUG_DFPUFP                 = 0x00,  /*!< Message originated from a DFP or UFP  */
  USBPD_CABLEPLUG_CABLEPLUG              = 0x01   /*!< Message originated from a Cable Plug  */
}USBPD_CablePlug_TypeDef;
/** 
  * @}
  */


/** @defgroup USBPD_PE_HandleTypeDef USB PD handle Structure definition
  * @brief  USB PD handle Structure definition
  * @{
  */
typedef struct
{
  uint32_t                        PE_ListOfPDO[USBPD_MAX_NB_PDO];         /*!< The list of supported Power Data Objects of the  on the current port Source or Sink  */
  uint32_t                        PE_NumberOfPDO;                         /*!< The number of supported Power Data Objects                                           
                                                                               This parameter must be set to a value lower than USBPD_MAX_NB_PDO                    */
  uint32_t                        PE_RequestDOMsg;                        /*!< Request Power Data Object message to be sent                                         */
  USBPD_SNKPowerRequest_TypeDef   PE_SNKRequestedPower;                   /*!< Request Power by the sink board                                                      */
  
  uint32_t                        PE_ListOfRcvPDO[USBPD_MAX_NB_PDO];      /*!< The list of received Power Data Objects from the port Partner                        */
  uint32_t                        PE_NumberOfRcvPDO;                      /*!< The number of received Power Data Objects from the port Partner                      
                                                                               This parameter must be set to a value lower than USBPD_MAX_NB_PDO                    */
  uint32_t                        PE_RcvRequestDOMsg;                     /*!< Received request Power Data Object message from the port Partner                     */
  uint32_t                        PE_RcvBistDOMsg;                        /*!< Received Bist Data Object message from the port Partner                              */
  uint32_t                        PE_RequestedVoltge;                     /*!< Requested voltage from port Partner (in mV)                                          */
  
  PE_SRCState_TypeDef             PE_SRCCurrentState;                     /*!< PE Source current state                                                              */
  PE_SRCState_TypeDef             PE_SRCPrevState;                        /*!< PE Source previous state                                                             */

  PE_SNKState_TypeDef             PE_SNKCurrentState;                     /*!< PE Source current state                                                              */
  PE_SNKState_TypeDef             PE_SNKPrevState;                        /*!< PE Sink previous state                                                               */
  
  USBPD_CMD_StateTypeDef          PE_CommandState;                        /*!< USB PD current command state                                                         */
  
  USBPD_PECounters_TypeDef        PE_Counters;                            /*!< PE Counters                                                                          */
  USBPD_PETimers_TypeDef          PE_Timers;                              /*!< PE Timers                                                                            */
  
  __IO uint8_t                    PE_CapReceived;                         /*!< Set when Sink receive Source_Capabilities Messages from the Source                   */
  __IO uint8_t                    PE_ReqReceived;                         /*!< Set when Source receive Request Messages from the Sink                               */
  
  __IO uint8_t                    PE_IsExplicitContract;                  /*!< Set to 1 if there is an Explicit Contract in place                                   */
  __IO uint8_t                    PE_RDOPosition;                         /*!< The position of the received request data object from sink port                      */
  __IO uint8_t                    PE_IsConnected;                         /*!< USB PD cable connection state                                                        */
  
  __IO uint8_t                    PE_SNKRDOPosition;                      /*!< The position of the requested power data object by sink port                         */
  
  __IO uint32_t                   PE_ErrorCode;                           /*!< USB PD Error code                                                                    */
  USBPD_PortPowerRole_TypeDef     PE_PortPowerRole;                       /*!< USB PD current port power role, Sink or Source                                       */
  USBPD_PortDataRole_TypeDef      PE_PortDataRole;                        /*!< USB PD current port Data role, DFP or UFP                                            */
  USBPD_PortPowerRole_TypeDef     PE_DefaultPortPowerRole;                /*!< USB PD default port power role, used in DRP mode                                     */
  __IO uint8_t                    PE_IsPortGiveBack;                      /*!< Set when current port supports GiveBack capabilities                                 */
  __IO uint32_t                   PE_gTimer;                              /*!< USB PD Global timer used to detect timeout error                                     */
  USBPD_MsgHeader_TypeDef         PE_MsgHeader;                           /*!< The header of message to be sent                                                     */
  USBPD_MsgType_TypeDef           PE_RxMsgType;                           /*!< The received message type: SOP, SOP', SOP''                                          */
  USBPD_MsgType_TypeDef           PE_TxMsgType;                           /*!< The Tx message type: SOP, SOP', SOP''                                                */
  uint8_t                         PE_RxBuffer[USBPD_MAX_RX_BUFFER_SIZE];  /*!< USB PD buffer used for reception                                                     */
  uint8_t                         PE_RxXferSize;                          /*!< Size of received data, must be lower than  USBPD_MAX_RX_BUFFER_SIZE                  */
  uint8_t                         PE_TxBuffer[USBPD_MAX_RX_BUFFER_SIZE];  /*!< USB PD buffer used for transmission                                                  */
  uint8_t                         PE_TxXferSize;                          /*!< Tx transfer size, must be lower than USBPD_MAX_TX_BUFFER_SIZE                        */
  USBPD_PE_Callbacks              PE_Callbacks;                           /*!< CallBacks exposed by the PE to the  DMP                                              */
  uint8_t                         PE_CurrentPortNumber;                   /*!< The current used port number                                                         */
}USBPD_PE_HandleTypeDef;
/** 
  * @}
  */


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
USBPD_StatusTypeDef USBPD_PE_Init(uint8_t portnum,USBPD_PortPowerRole_TypeDef role, USBPD_PE_Callbacks pecallbacks);
USBPD_StatusTypeDef USBPD_PE_SRCInit(uint8_t portnum, USBPD_PortPowerRole_TypeDef role, USBPD_PE_Callbacks pecallbacks);
USBPD_StatusTypeDef USBPD_PE_SNKInit(uint8_t portnum, USBPD_PortPowerRole_TypeDef role, USBPD_PE_Callbacks pecallbacks);
USBPD_StatusTypeDef USBPD_PE_DeInit(uint8_t portnum);
USBPD_StatusTypeDef USBPD_PE_SRCStateMachine(uint8_t portnum);
USBPD_StatusTypeDef USBPD_PE_SNKStateMachine(uint8_t portnum);
void USBPD_PE_SRCProcess(uint8_t portnum);
void USBPD_PE_SNKProcess(uint8_t portnum);
void USBPD_PE_DRPProcess(uint8_t portnum);
void USBPD_PE_RxProcess(uint8_t portnum, USBPD_MsgType_TypeDef msgtype);
USBPD_StatusTypeDef USBPD_PE_SetPowerProfile(uint8_t portnum, uint32_t *pPDO, uint32_t nbpdo);
USBPD_StatusTypeDef USBPD_PE_SetSNKRequiredPower(uint8_t portnum, uint32_t current, uint32_t voltage, uint32_t maxvoltage, uint32_t minvoltage);
USBPD_StatusTypeDef USBPD_PE_GetReceivedPowerProfile(uint8_t portnum, uint32_t *pPDO, uint32_t *nbpdo);
USBPD_PortPowerRole_TypeDef USBPD_PE_GetPowerRole(USBPD_PE_HandleTypeDef *pdhandle);
USBPD_PortPowerRole_TypeDef USBPD_PE_DefaultPortPowerRolee(USBPD_PE_HandleTypeDef *pdhandle);
void USBPD_PE_ChangePowerRole(USBPD_PE_HandleTypeDef *pdhandle, USBPD_PortPowerRole_TypeDef newrole);
USBPD_StatusTypeDef USBPD_PE_IsCableConnected(uint8_t portnum, uint8_t isconnected);

/* PE Timer functions */
void USBPD_PE_TimerCounter(uint8_t portnum);
void USBPD_PE_TimerProcess(uint8_t portnum);

/*******************************************************************************
                              Power Negotiation
*******************************************************************************/
/* Send Control messages */
USBPD_StatusTypeDef USBPD_PE_SendControlMsg(USBPD_PE_HandleTypeDef *pdhandle, USBPD_ControlMsg_TypeDef msgtype);

/* Capabilities Message */
USBPD_StatusTypeDef USBPD_PE_SendCapabilities(USBPD_PE_HandleTypeDef *pdhandle, uint32_t *pPDO, uint32_t nbpdo);
USBPD_StatusTypeDef USBPD_PE_CapabilitiesReceived(USBPD_PE_HandleTypeDef *pdhandle);
USBPD_StatusTypeDef USBPD_PE_CapabilitiesSent(USBPD_PE_HandleTypeDef *pdhandle);
USBPD_StatusTypeDef USBPD_PE_EvaluateCapabilities(USBPD_PE_HandleTypeDef *pdhandle);

/* Request Message */
USBPD_StatusTypeDef USBPD_PE_SendRequest(USBPD_PE_HandleTypeDef *pdhandle, uint32_t *pRDO);
USBPD_StatusTypeDef USBPD_PE_RequestReceived(USBPD_PE_HandleTypeDef *pdhandle);
USBPD_StatusTypeDef USBPD_PE_EvaluateRequest(USBPD_PE_HandleTypeDef *pdhandle);
USBPD_StatusTypeDef USBPD_PE_RequestNewPowerProfile(uint8_t portnum, uint8_t pdoindex);

/* Accept Message */
USBPD_StatusTypeDef USBPD_PE_AcceptReceived(USBPD_PE_HandleTypeDef *pdhandle);

/* Reject Message */
USBPD_StatusTypeDef USBPD_PE_RejectReceived(USBPD_PE_HandleTypeDef *pdhandle);

/* PS_RDY Message */
USBPD_StatusTypeDef USBPD_PE_PSRDYReceived(USBPD_PE_HandleTypeDef *pdhandle);

/* Wait Message */
USBPD_StatusTypeDef USBPD_PE_WaitReceived(USBPD_PE_HandleTypeDef *pdhandle);

/* DRP Message */
USBPD_StatusTypeDef USBPD_PE_SrcSnkPRSwapProcess(uint8_t portnum);
USBPD_StatusTypeDef USBPD_PE_SnkSrcPRSwapProcess(uint8_t portnum);
USBPD_StatusTypeDef USBPD_PE_RequestPowerRoleSwap(uint8_t portnum);

/* BIST */
USBPD_StatusTypeDef USBPD_PE_BistReceived(USBPD_PE_HandleTypeDef *pdhandle);

/*******************************************************************************
                              Soft Reset
*******************************************************************************/
USBPD_StatusTypeDef USBPD_PE_SoftResetReceived(USBPD_PE_HandleTypeDef *pdhandle);
USBPD_StatusTypeDef USBPD_PE_PerformSoftReset(USBPD_PE_HandleTypeDef *pdhandle);

/*******************************************************************************
                          Source and Sink Hard Reset
*******************************************************************************/
USBPD_StatusTypeDef USBPD_PE_SendHardReset(uint8_t portnum);
void USBPD_PE_HardResetReceived(uint8_t portnum, USBPD_MsgType_TypeDef msgtype);

#ifdef __cplusplus
}
#endif

#endif /* __USBPD_PE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
