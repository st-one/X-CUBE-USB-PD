/**
  **************************************************************************************************
  * @file    STUSB1602_Driver.h
  * @author  System Lab - Sensing & Connectivity Application Team
  * @brief   This file provides a set of structs and unions needed to manage the STUSB1602 Driver.
  **************************************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  *    may be used to endorse or promote products derived from this software
  *    without specific prior written permission.
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
  **************************************************************************************************
  */

#ifndef __STUSB1602_DRIVER_H_
#define __STUSB1602_DRIVER_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/


/** @addtogroup STM32_USBPD_LIBRARY
  * @{
  */

/** @addtogroup USBPD_DEVICE
 * @{
 */

/** @addtogroup STUSB1602_LIBRARY
 * @{
 */

/* Exported defines ----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Exported_Defines USBPD DEVICE STUSB1602 LIBRARY Exported defines
* @{
*/
  
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_I2C_Defines USBPD DEVICE STUSB1602 LIBRARY I2C defines
* @{
*/
#define STUSB1602_I2C_Add_0     					0x28 	/*<! Address 0 */
#define STUSB1602_I2C_Add_1     					0x29	/*<! Address 1 */
#define TIMEOUT_MAX             2000 /*<! The value of the maximal timeout for BUS waiting loops */

/**
 * @}
 */
  
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Masks_Defines USBPD DEVICE STUSB1602 LIBRARY Masks defines
* @{
*/
#define STUSB16xx_FAULT_STATUS_AL_MASK          0x08;
#define STUSB16xx_MONITORING_STATUS_AL_MASK     0x10;
#define STUSB16xx_CC_DETECTION_STATUS_AL_MASK   0x20;
/**
 * @}
 */

/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Registers_Map_Defines USBPD DEVICE STUSB1602 LIBRARY Registers map defines
* @{
*/
#define STUSB1602_ALERT_STATUS_REG				0x0B
#define STUSB1602_ALERT_STATUS_MASK_REG				0x0C
#define STUSB1602_CC_DETECTION_STATUS_TRANS_REG		        0x0D
#define STUSB1602_CC_DETECTION_STATUS_REG			0x0E
#define STUSB1602_MONITORING_STATUS_TRANS_REG		        0x0F
#define STUSB1602_MONITORING_STATUS_REG				0x10
#define STUSB1602_CC_CONNECTION_STATUS_REG			0x11
#define STUSB1602_HW_FAULT_STATUS_TRANS_REG			0x12
#define STUSB1602_HW_FAULT_STATUS_REG				0x13
#define STUSB1602_PHY_STATUS_REG        			0x17
#define STUSB1602_CC_CAPABILITY_CTRL_REG			0x18
#define STUSB1602_CC_VCONN_SWITCH_CTRL_REG			0x1E
#define STUSB1602_CC_MODE_CTRL_REG				0x1F
#define STUSB1602_VCONN_MONITORING_CTRL_REG			0x20
#define STUSB1602_VBUS_SELECT_REG		        	0x21
#define STUSB1602_VBUS_RANGE_MONITORING_CTRL_REG	        0x22
#define STUSB1602_RESET_CTRL_REG				0x23
#define STUSB1602_CC_POWERED_ACCESSORY_CTRL_REG		        0x24
#define STUSB1602_VBUS_DISCHARGE_TIME_CTRL_REG		        0x25
#define STUSB1602_VBUS_DISCHARGE_CTRL_REG			0x26
#define STUSB1602_VBUS_ENABLE_STATUS_REG			0x27
#define STUSB1602_MODE_CTRL_REG			        	0x28
#define STUSB1602_VBUS_MONITORING_CTRL_REG			0x2E
#define STUSB1602_DEVICE_CUT_REG		        	0x2F
/**
 * @}
 */

/**
 * @}
 */
  
  
/* Exported typedef ----------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Exported_Types USBPD DEVICE STUSB1602 LIBRARY Exported types
* @{
*/

/**
 * @brief  STUSB1602 Status
  */
typedef enum
{
	STUSB1602_OK       = 0x00,		/*!< Device OK 		*/
	STUSB1602_ERROR    = 0x01,		/*!< Device ERROR 	*/
	STUSB1602_BUSY     = 0x02,		/*!< Device BUSY 	*/
	STUSB1602_TIMEOUT  = 0x03		/*!< Device TIMEOUT */
} STUSB1602_StatusTypeDef;

/** @defgroup STUSB1602_REGISTERS_TYPES STUSB1602 Registers types
 * @brief STUSB1602 Register structure definitions
* @{
*/

/**
  * @brief  	STUSB1602 ALERT_STATUS register Structure definition
 * @Address     0Bh
 * @Access 	RC
 * @details 	This register indicates that an Alert has occurred.\n
  *             (When a bit value change occurs on one of the mentioned transition register, it automatically
  *             sets the corresponding alert bit in ALERT_STATUS register. )
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t _Reserved_0_3               :       4;
    uint8_t HW_FAULT_STATUS_AL          :       1;
    uint8_t MONITORING_STATUS_AL        :       1;
    uint8_t CC_DETECTION_STATUS_AL      :       1;
    uint8_t _Reserved_7                 :       1;
  } b;
} STUSB1602_ALERT_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 ALERT_STATUS_MASK register Structure definition
 * @Address 	0Ch
 * @Access 	R/W
 * @details 	This register is used to mask event interrupt and prevent the assertion of the alert bit in the
  *		ALERT_STATUS register when the corresponding bit defined below is set to 1.
  *		The condition for generating an active-low ALERT signal is:
  *		[ALERT_STATUS bitwise AND (NOT ALERT_STATUS_MASK)] <> 0
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t _Reserved_0_3 			:       4;
    uint8_t HW_FAULT_STATUS_AL_MASK 	        :       1;
    uint8_t MONITORING_STATUS_AL_MASK 	        :       1;
    uint8_t CC_DETECTION_STATUS_AL_MASK         :       1;
    uint8_t _Reserved_7 			:       1;
  } b;
} STUSB1602_ALERT_STATUS_MASK_reg_TypeDef;

/**
 * @brief  	STUSB1602 CC_DETECTION_STATUS_TRANS register Structure definition
 * @Address 	0Dh
 * @Access 	RC
 * @details 	This register indicates a bit value change has occurred in CC_DETECTION_STATUS register.
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t ATTACH_STATE_TRANS 	        :       1;
    uint8_t _Reserved_1_7		:       7;
  } b;
} STUSB1602_CC_DETECTION_STATUS_TRANS_RegTypeDef;

/**
 * @brief  	STUSB1602 CC_DETECTION_STATUS register Structure definition
 * @Address 	0Eh
 * @Access 	RO
 * @details 	This register provides current status of the connection detection and corresponding operation modes.
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t CC_ATTACH_STATE			:       1;
    uint8_t CC_VCONN_SUPPLY_STATE 		:       1;
    uint8_t CC_DATA_ROLE 			:       1;
    uint8_t CC_POWER_ROLE 			:       1;
    uint8_t START_UP_POWER_MODE 		:       1;
    uint8_t CC_ATTACH_MODE 			:       3;
  } b;
} STUSB1602_CC_DETECTION_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 MONITORING_STATUS_TRANS register Structure definition
 * @Address 	0Fh
 * @Access 	RC
 * @details 	This register allows to:
  *              - Alert about any change that occurs in MONITORING_STATUS register.
  *              - Manage specific USB PD Acknowledge commands
  *              - to manage Type-C state machine Acknowledge to USB PD Requests commands
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t VCONN_PRESENCE_TRANS 	:       1;
    uint8_t VBUS_PRESENCE_TRANS		:       1;
    uint8_t VBUS_VSAFE0V_TRANS 	        :       1;
    uint8_t VBUS_VALID_TRANS 	        :       1;
    uint8_t PD_TYPEC_HAND_SHAKE		:       4;
  } b;
} STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef;

/**
 * @brief  	STUSB1602 MONITORING_STATUS register Structure definition
 * @Address 	10h
 * @Access 	RO
 * @details 	This register provides information on current status of the VBUS and VCONN voltages
  *		monitoring done respectively on VBUS_SENSE input pin and VCONN input pin.
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t VCONN_PRESENCE		 	: 	1;
    uint8_t VBUS_PRESENCE			: 	1;
    uint8_t VBUS_VSAFE0V	 		: 	1;
    uint8_t VBUS_VALID 			        : 	1;
    uint8_t _Reserved_4_7			:	4;
  } b;
} STUSB1602_MONITORING_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 CC_CONNECTION_STATUS register Structure definition
 * @Address 	11h
 * @Access 	RO
 * @details 	This register provides information on the connection state with respect to the Type-C FSM states
  *		as defined in the USB Type-C standard specification. This status is informative only and is not used
  *		to trigger any alert.
  *		The reset value of TYPEC_FSM_STATE bits is:
  *		 - 00h when device is operating in Sink power role (Unattached.SNK)
  *		 - 08h when device is operating in Source power role (Unattached.SRC)
  *
  *             The CC_ATTACHED bit provides cable orientation by indicating which CC pin is connected to the CC line.
  *             Its value is consistent with the logic level of the A_B_SIDE output pin providing similar information.
  *
  *             The SINK_POWER_STATE bits indicate the current level advertised by the Source that the Sink can
  *             consume when the device is operating in Sink power role.
  *
  *             The TYPEC_FSM_STATE bits indicate the current state of the Type-C FSM corresponding to the power
  *             mode defined in CC_MODE_CTRL register.
 */
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t TYPEC_FSM_STATE 	 	: 	5;
    uint8_t SINK_POWER_STATE 		: 	2;
    uint8_t CC_ATTACHED			:	1;
  } b;
} STUSB1602_CC_CONNECTION_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 HW_FAULT_STATUS_TRANS register Structure definition
 * @Address 	12h
 * @Access 	RC
 * @details 	This register indicates a bit value change has occurred in HW_FAULT_STATUS register.
  *		It alerts also when the over temperature condition is met.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VCONN_SW_OVP_FAULT_TRANS	: 	1;
        uint8_t VCONN_SW_OCP_FAULT_TRANS	:	1;
        uint8_t VCONN_SW_RVP_FAULT_TRANS	: 	1;
        uint8_t _Reserved_3			: 	1;
        uint8_t VPU_PRESENCE_TRANS		:	1;
        uint8_t VPU_OVP_FAULT_TRANS 	        : 	1;
        uint8_t _Reserved_6			: 	1;
        uint8_t THERMAL_FAULT		        :	1;
  } b;
} STUSB1602_HW_FAULT_STATUS_TRANS_RegTypeDef;

/**
 * @brief  	STUSB1602 HW_FAULT_STATUS register Structure definition
 * @Address 	13h
 * @Access 	RO
 * @details 	This register provides information on hardware fault conditions related to the
  *		internal pull-up voltage in Source power role and to the VCONN power switches
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VCONN_SW_OVP_FAULT_CC2		:	1;
        uint8_t VCONN_SW_OVP_FAULT_CC1		:	1;
        uint8_t VCONN_SW_OCP_FAULT_CC2		:	1;
        uint8_t VCONN_SW_OCP_FAULT_CC1		:	1;
        uint8_t VCONN_SW_RVP_FAULT_CC2		:	1;
        uint8_t VCONN_SW_RVP_FAULT_CC1		:	1;
        uint8_t VPU_PRESENCE			:	1;
        uint8_t VPU_OVP_FAULT			:	1;
  } b;
} STUSB1602_HW_FAULT_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 PHY_STATUS register Structure definition
 * @Address 	17h
 * @Access 	RC
 * @details 	This register provides information on hardware PHy status condition
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t _Reserved_0_2		        :	3;
        uint8_t BUS_IDLE		        :	1;    /*this is alway 0 in cut 1.2)*/
        uint8_t _Reserved_4_7   		:	4;
  } b;
} PHY_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 CC_CAPABILITY_CTRL register Structure definition
 * @Address 	18h
 * @Access 	R/W
 * @details 	This register allows to change the advertising of the current capability and the VCONN
  *		supply capability when operating in Source power role.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t CC_VCONN_SUPPLY_EN		:	1;
        uint8_t VCONN_SWAP_EN		        :	1;
        uint8_t PR_SWAP_EN		        :	1;
        uint8_t DR_SWAP_EN		        :	1;
        uint8_t CC_VCONN_DISCHARGE_EN	        :	1;
        uint8_t SNK_DISCONNECT_MODE		:	1;
        uint8_t CC_CURRENT_ADVERTISED	        :	2;
  } b;
} STUSB1602_CC_CAPABILITY_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 CC_VCONN_SWITCH_CTRL register Structure definition
 * @Address 	1Eh
 * @Access 	R/W
 * @details 	This register allows to change the default current limit of the power switches
  *		supplying VCONN on the CC pins.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t CC_VCONN_SWITCH_ILIM	        :	4;
        uint8_t _Reserved_4_7		        :	4;
  } b;
} STUSB1602_CC_VCONN_SWITCH_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 CC_MODE_CTRL register Structure definition
 * @Address 	1Fh
 * @Access 	R/W
 * @details 	This register allows:
  *		- to manage specific USB PD Requests commands to Type-C state machine.
 */
typedef union
{
  uint8_t d8;
  struct
  {

    uint8_t _Reserved_0_3		:	4;
    uint8_t TYPEC_CTRL			:	4;
  } b;
} STUSB1602_CC_MODE_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 VCONN_MONITORING_CTRL register Structure definition
 * @Address 	20h
 * @Access 	R/W
 * @details 	This register allows to modify the default voltage monitoring conditions for VCONN.
  *
 * @note Disabling the UVLO threshold detection on VCONN pin is deactivating the VCONN power path
  *		and setting VCONN_PRESENCE bit to 0b in MONITORING_STATUS register.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t _Reserved_0_3		        :	4;
        uint8_t _Reserved_4		        :	1;
        uint8_t _Reserved_5		        :	1;
        uint8_t VCONN_UVLO_THRESHOLD	        :	1;
        uint8_t VCONN_MONITORING_EN		:	1;
  } b;
} STUSB1602_VCONN_MONITORING_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 VBUS_SELECT register Structure definition
 * @Address 	21h
 * @Access 	R/W
 * @details 	This register contains the parameters used for VBUS monitoring voltage.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VBUS_SELECT		        :	8;
  } b;
} STUSB1602_VBUS_SELECT_RegTypeDef;

/**
 * @brief  	STUSB1602 VBUS_RANGE_MONITORING_CTRL register Structure definition
 * @Address 	22h
 * @Access 	R/W
 * @details 	This register contains the parameters used to shift the VBUS monitoring voltage range.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VBUS_VSHIFT_LOW		        :	4;
        uint8_t VBUS_VSHIFT_HIGH		:	4;
  } b;
} STUSB1602_VBUS_RANGE_MONITORING_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 RESET_CTRL register Structure definition
 * @Address 	23h
 * @Access 	R/W
 * @details 	This register allows to reset the device by software.
  *		The SW_RESET_EN bit acts as the hardware RESET pin but it does not command the RESET pin.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t SW_RESET_EN			:	1;
        uint8_t _Reserved_1_7		        :	7;
  } b;
} STUSB1602_RESET_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 CC_POWERED_ACCESSORY_CTRL register Structure definition
 * @Address 	24h
 * @Access 	R/W
 * @details 	This register controls the powered accessory detection and operation modes when the device
  *		is configured in Sink power role with accessory support (see Section 4.4.12: CC_MODE_CTRL).
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t PWR_ACC_DETECT_EN	        :	1;
        uint8_t PWR_ACC_TRY_SNK_EN		:	1;
        uint8_t _Reserved_3_7		        :	6;
  } b;
} STUSB1602_CC_POWERED_ACCESSORY_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 VBUS_DISCHARGE_TIME_CTRL register Structure definition
 * @Address 	25h
 * @Access 	R/W
 * @details 	This register contains the parameter used to define the VBUS discharge time when the VBUS discharge path is activated).
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VBUS_DISCHARGE_TIME_TO_PDO      :	4;
        uint8_t VBUS_DISCHARGE_TIME_TO_0V	:	4;
  } b;
} STUSB1602_VBUS_DISCHARGE_TIME_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 VBUS_DISCHARGE_CTRL register Structure definition
 * @Address 	26h
 * @Access 	R/W
 * @details 	This register controls the activation of the VBUS discharge path.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t _Reserved_0_6	                :	7;
        uint8_t VBUS_DISCHARGE_EN	        :	1;
  } b;
} STUSB1602_VBUS_DISCHARGE_CTRL_RegTypeDef;

/**
 * @brief  	STUSB1602 VBUS_ENABLE_STATUS register Structure definition
 * @Address 	27h
 * @Access 	RO
 * @details 	This register provides information on current status of the VBUS power path activation
  *		through VBUS_EN_SRC pin in Source power role and VBUS_EN_SNK pin in Sink power role.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VBUS_SOURCE_EN		        :	1;
        uint8_t VBUS_SINK_EN		        :	1;
        uint8_t _Reserved_2_7		        :	6;
  } b;
} STUSB1602_VBUS_ENABLE_STATUS_RegTypeDef;

/**
 * @brief  	STUSB1602 MODE_CTRL register Structure definition
 * @Address 	28h
 * @Access 	R/W
 * @details 	This register allows:
    *             - to change the default Type-C power mode if needed. It requires that the hardware implementation
    *               of the targeted application is consistent with the functioning of the new Type-C power mode selected.
 */
  typedef union
  {
    uint8_t d8;
    struct
    {
          uint8_t POWER_MODE			:	3;
          uint8_t _Reserved_0			:	5;
    } b;
  } STUSB1602_MODE_CTRL_RegTypeDef;

/**
 * @brief   	STUSB1602 VBUS_MONITORING_CTRL register Structure definition
 * @Address 	2Eh
 * @Access 	R/W
 * @details 	This register allows to modify the default monitoring conditions of VBUS voltage over the
  *		power path from VDD and VBUS_SENSE pins.
 */
typedef union
{
  uint8_t d8;
  struct
  {
	uint8_t VDD_UVLO_DISABLE		:	1;
	uint8_t VBUS_VSAFE0V_THRESHOLD	        :	2;
	uint8_t _Reserved_3			:	1;
	uint8_t VBUS_RANGE_DISABLE		:	1;
	uint8_t _Reserved_5			:	1;
	uint8_t VDD_OVLO_DISABLE		:	1;
        uint8_t _Reserved_7			:	1;
  } b;
} STUSB1602_VBUS_MONITORING_CTRL_RegTypeDef;

/**
 * @brief  	CC and monitoring status fast detection
 * @Address 	0Bh to 10h
 * @Access 	R
 * @details 	This register allows to modify the default monitoring conditions of VBUS voltage over the
  *		power path from VDD and VBUS_SENSE pins.
 */
typedef struct
{
	STUSB1602_ALERT_STATUS_RegTypeDef                     reg_0B;   /*!< STUSB1602 ALERT_STATUS register */ 
	STUSB1602_ALERT_STATUS_MASK_reg_TypeDef               reg_0C;   /*!< STUSB1602 ALERT_STATUS_MASK register */
	STUSB1602_CC_DETECTION_STATUS_TRANS_RegTypeDef        reg_0D;   /*!< STUSB1602 CC_DETECTION_STATUS_TRANS register */
	STUSB1602_CC_DETECTION_STATUS_RegTypeDef              reg_0E;   /*!< STUSB1602 CC_DETECTION_STATUS register */
	STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef          reg_0F;   /*!< STUSB1602 MONITORING_STATUS_TRANS register */
	STUSB1602_MONITORING_STATUS_RegTypeDef                reg_10;   /*!< STUSB1602 MONITORING_STATUS register */
	STUSB1602_CC_CONNECTION_STATUS_RegTypeDef             reg_11;   /*!< STUSB1602 CC_CONNECTION_STATUS register */
	STUSB1602_HW_FAULT_STATUS_TRANS_RegTypeDef            reg_12;   /*!< STUSB1602 HW_FAULT_STATUS_TRANS register */
} STUSB1602_ALERT_MONITORING_TypeDef;

/**
 * @brief  	alert source status 
 * @Address 	0Bh to 0Ch
 * @Access 	R
 * @details 	This register allows to determine the source of the interrupt 
  *	
 */
typedef struct
{
	STUSB1602_ALERT_STATUS_RegTypeDef                     reg_0B;   /*!< STUSB1602 ALERT_STATUS register */ 
	STUSB1602_ALERT_STATUS_MASK_reg_TypeDef               reg_0C;   /*!< STUSB1602 ALERT_STATUS_MASK register */
} STUSB1602_ALERT_RAISE_TypeDef;
/**
 * @}
 */

/** @defgroup STUSB1602_REGISTERS_VALUES STUSB1602 Register values
 * @brief STUSB1602 Register values enumerations
* @{
*/
/*0x0B*/

/*0x0C*/
/**
  * @brief STUSB1602 CC_DETECTION_STATUS_AL_MASK
 * @Address 0Ch - Bit6
 * @Access R/W
  */
typedef enum
{
  CC_Detect_Int_UNMASKED            = 0,    /*!< DEFAULT: Interrupt unmasked  */
  CC_Detect_Int_MASKED              = 1     /*!< Interrupt masked           */
} CC_Detect_Alrt_Int_Mask_Status_TypeDef;

/**
  * @brief STUSB1602 MONITORING_STATUS_AL_MASK
 * @Address 0Ch - Bit5
 * @Access R/W
  */
typedef enum
{
  Monitor_Status_Int_UNMASKED            = 0,    /*!< DEFAULT: Interrupt unmasked  */
  Monitor_Status_Int_MASKED              = 1     /*!< Interrupt masked           */
} Monitor_Alrt_Int_Mask_Status_TypeDef;

/**
  * @brief STUSB1602 HW_FAULT_STATUS_AL_MASK
 * @Address 0Ch - Bit4
 * @Access R/W
  */
typedef enum
{
  HW_Fault_Int_UNMASKED            = 0,    /*!< DEFAULT: Interrupt unmasked  */
  HW_Fault_Int_MASKED              = 1     /*!< Interrupt masked           */
} HW_Fault_Alrt_Int_Mask_Status_TypeDef;

/*0x0D*/
/**
  * @brief STUSB1602 ATTACH STATE TRANS
 * @Address 0Dh - Bit0
 * @Access RC
  */
typedef enum
{
  No_Attach_Transition                       = 0,    /*!< DEFAULT: Nothing attached  */
  Attach_Transition_Occurred                 = 1     /*!< Device attached            */
} Attach_State_Trans_TypeDef;

/*0x0E*/
/**
  * @brief STUSB1602 CC Attach State
 * @Address 0Eh - Bit0
 * @Access RO
  */
typedef enum
{
  Device_Detached                         = 0,    /*!< DEFAULT: Nothing attached  */
  Device_Attached                         = 1     /*!< Device attached            */
} Attach_State_TypeDef;

/**
  * @brief STUSB1602 CC VCONN Supply State
 * @Address 0Eh - Bit1
 * @Access RO
  */
typedef enum
{
  VCONN_not_supplied                      = 0,    /*!< DEFAULT: Device is not supplying VCONN                   */
  VCONN_supplied_on_unused_CC_pin         = 1     /*!< Device is supplying VCONN on the unused CC pin           */
} VCONN_Supply_State_TypeDef;

/**
  * @brief STUSB1602 Data Role
 * @Address 0Eh - Bit2
 * @Access RO
  */
typedef enum
{
  UFP_data_mode                 = 0,    /*!< DEFAULT: Device is operating in UFP data mode         */
  DFP_data_mode                 = 1     /*!< Device is operating in DFP data mode       */
} Data_Role_TypeDef;

/**
  * @brief STUSB1602 Power Role
 * @Address 0Eh - Bit3
 * @Access RO
  */
typedef enum
{
  Sink                          = 0,    /*!< DEFAULT: Device is operating in Sink power role         */
  Source                        = 1     /*!< Device is operating in Source power role       */
} Power_Role_TypeDef;

/**
  * @brief STUSB1602 Start Up Mode
 * @Address 0Eh - Bit4
 * @Access RO
  */
typedef enum
{
  Normal_Mode                           = 0,    /*!< DEFAULT: Device is starting in normal mode         */
  Standby_Mode                          = 1     /*!< Device is starting in standby mode        */
} StartUp_Mode_TypeDef;

/**
  * @brief STUSB1602 CC Attach modes
 * @Address 0Eh - Bit7:5
 * @Access RO
  */
typedef enum
{
  Nothing_Attached                   = 0x00,    /*!< DEFAULT: Nothing attached  */
  Sink_Attached                      = 0x01,    /*!< Sink attached              */
  Source_Attached                    = 0x02,    /*!< Source attached            */
  Debug_Acc_Attached                 = 0x03,    /*!< Debug Accessory attached   */
  Audio_Acc_Attached                 = 0x04,    /*!< Audio Accessory attached   */
  Powered_Acc_Attached               = 0x05     /*!< Powered Accessory attached */
} Attach_Mode_TypeDef;

/*0x0F*/
/**
  * @brief STUSB1602 PD Type C Hand shake Ack
 * @Address 0Fh - Bit7:4
 * @Access RC
  */
typedef enum
{
  TypeC_HS_Cleaned                      = 0x00,    /*!< DEFAULT: Cleared                  */
  TypeC_NoAck                           = TypeC_HS_Cleaned,
  PD_PR_Swap_Ps_Rdy_Ack                 = 0x01,    /*!< PD_PR_PS_RDY_ACK */
  PD_PR_SWAP_Rp_Assert_Ack              = 0x02,    /*!< PD_PR_SWAP_RP_ASSERT_ACK           */
  PD_PR_SWAP_Rd_Assert_Ack              = 0x03,    /*!< PD_PR_SWAP_RD_ASSERT_ACK   */
  PD_DR_SWAP_Port_Change_2_DFP_Ack      = 0x04,    /*!< PD_DR_SWAP_PORT_CHANGE_2_DFP_ACK   */
  PD_DR_SWAP_Port_Change_2_UFP_Ack      = 0x05,    /*!< PD_DR_SWAP_PORT_CHANGE_2_UFP_ACK */
  PD_VCONN_SWAP_Turn_On_VCONN_Ack       = 0x06,    /*!< PD_VCONN_SWAP_TURN_ON_VCONN_ACK */
  PD_VCONN_SWAP_Turn_Off_VCONN_Ack      = 0x07,     /*!< PD_VCONN_SWAP_TURN_OFF_VCONN_ACK */
  PD_Hard_Reset_Complete_Ack            = 0x08,    /*!< PD_HARD_RESET_COMPLETE_ACK */
  PD_Hard_Reset_Turn_Off_Vconn_Ack      = 0x09,    /*!< PD_HARD_RESET_TURN_OFF_VCONN_ACK */
  PD_Hard_Reset_Port_Change_2_DFP_Ack   = 0x0A,    /*!< PD_HARD_RESET_PORT_CHANGE_2_DFP_ACK */
  PD_Hard_Reset_Port_Change_2_UFP_Ack   = 0x0B,    /*!< PD_HARD_RESET_PORT_CHANGE_2_UFP_ACK */
  PD_PR_Swap_Snk_Vbus_Off_Ack           = 0x0C,    /*!< PD_PR_SWAP_SNK_VBUS_OFF_ACK */
  PD_PR_Swap_Src_Vbus_Off_Ack           = 0x0D,    /*!< PD_PR_SWAP_SRC_VBUS_OFF_ACK */
  PD_Hard_Reset_Received_Ack            = 0x0E,    /*!< PD_HARD_RESET_RECEIVED_ACK */
  PD_Hard_Reset_Send_Ack                = 0x0F     /*!< PD_HARD_RESET_SEND_ACK */

} PD_TypeC_Handshake_TypeDef;

/**
  * @brief STUSB1602 VBUS_Valid_Trans
 * @Address 0Fh - Bit3
 * @Access RC
  */
typedef enum
{
  No_VBUS_VALID_Transition              = 0,            /*!< DEFAULT: Cleared        */
  VBUS_VALID_Transition_Occurred        = 1             /*!< Transition occured on VBUS_VALID bit        */
} VBUS_Valid_Trans_TypeDef;

/**
  * @brief STUSB1602 VBUS_VSAFE0V_Trans
 * @Address 0Fh - Bit2
 * @Access RC
  */
typedef enum
{
  No_VBUS_VSAFE0V_Transition                   = 0,            /*!< Cleared        */
  VBUS_VSAFE0V_Transition_Occurred             = 1             /*!< DEFAULT: Transition occured on VBUS_VSAFE0V bit  */
}VBUS_VSAFE0V_Trans_TypeDef;

/**
  * @brief STUSB1602 VBUS_Presence_Trans
 * @Address 0Fh - Bit1
 * @Access RC
  */
typedef enum
{
  No_VBUS_Presence_Transition                   = 0,            /*!< DEFAULT: Cleared    */
  VBUS_Presence_Transition_Occurred             = 1             /*!< Transition occured on VBUS_Presence bit  */
}VBUS_Presence_Trans_TypeDef;

/**
  * @brief STUSB1602 VCONN_Presence_Trans
 * @Address 0Fh - Bit0
 * @Access RC
  */
typedef enum
{
  No_VCONN_Presence_Transition                  = 0,            /*!< DEFAULT: Cleared        */
  VCONN_Presence_Transition_Occurred            = 1             /*!< Transition occured on VCONN_Presence bit  */
}VCONN_Presence_Trans_TypeDef;

/*0x10*/
/**
  * @brief STUSB1602 VBUS_Valid
 * @Address 10h - Bit3
 * @Access RO
  */
typedef enum
{
  VBUS_outside_VALID_vrange         = 0,            /*!< DEFAULT: VBUS is outside valid voltage range        */
  VBUS_within_VALID_vrange          = 1             /*!< VBUS is within valid voltage range         */
} VBUS_Valid_TypeDef;

/**
  * @brief STUSB1602 VBUS_VSAFE0V
 * @Address 10h - Bit2
 * @Access RO
  */
typedef enum
{
  VBUS_above_VSAFE0V_threshold       = 0,            /*!< VBUS is above vSafe0V threshold         */
  VBUS_below_VSAFE0V_threshold       = 1             /*!< DEFAULT: VBUS is below VSafe0V threshold         */
}VBUS_VSAFE0V_TypeDef;

/**
  * @brief STUSB1602 VBUS_Presence
 * @Address 10h - Bit1
 * @Access RO
  */
typedef enum
{
  VBUS_below_UVLO_threshold        = 0,            /*!< DEFAULT: VBUS is below UVLO threshold       */
  VBUS_above_UVLO_threshold        = 1             /*!< VBUS is above UVLO threshold       */
}VBUS_Presence_TypeDef;

/**
  * @brief STUSB1602 VCONN_Presence
 * @Address 10h - Bit0
 * @Access RO
  */
typedef enum
{
  VCONN_below_UVLO_threshold        = 0,            /*!< DEFAULT: VCONN is below UVLO threshold     */
  VCONN_above_UVLO_threshold        = 1             /*!< DEFAULT: VCONN is above UVLO threshold     */
}VCONN_Presence_TypeDef;

/*0x11*/
/**
 * @brief USB PD CC lines
 * @Address 11h - Bit7
 * @Access RO
  */
typedef enum
{
	CC1Pin         = 0,                             /*!< DEFAULT: Type-C Configuration Channel 1 */
	CC2Pin         = 1,			        /*!< Type-C Configuration Channel 2 */
} CCxPin_Attached_TypeDef;

/**
 * @brief USB PD SINK_POWER_STATE
 * @Address 11h - Bit6:5
 * @Access RO
  */
typedef enum
{
  PwrDefault_SNK      = 0,              /*!< DEFAULT: PowerDefault.SNK (Source supplies Default USB current)    */
  Pwr_1_5_SNK         = 1,              /*!< Power1.5.SNK (Source supplies 1.5 A USB Type-C current)            */
  Pwr_3_0_SNK         = 2               /*!< Power3.0.SNK (Source supplies 3.0 A USB Type-C current)            */
} Sink_Power_State_TypeDef;

/**
 * @brief USB PD Type-C FSM State
 * @Address 11h - Bit4:0
 * @Access RO
  */
typedef enum
{
  Unattached_SNK                        =  0,   /*!< DEFAULT */
  AttachWait_SNK                        =  1,
  Attached_SNK                          =  2,
  DebugAccessory_SNK                    =  3,
  SNK_2_SRC_PR_SWAP                     =  6,
  TryWait_SNK                           =  7,
  Unattached_SRC                        =  8,   /*!< DEFAULT */
  AttachWait_SRC                        =  9,
  Attached_SRC                          = 10,
  SRC_2_SNK_PR_SWAP                     = 11,
  Try_SRC                               = 12,
  Unattached_Accessory                  = 13,
  AttachWait_Accessory                  = 14,
  AudioAccessory                        = 15,
  UnorientedDebugAccessory_SRC          = 16,
  Powered_Accessory                     = 17,
  Unsupported_Accessory                 = 18,
  ErrorRecovery                         = 19,
	TryDebounce_SNK                       = 20, 
  Try_SNK                               = 21,
  TryWait_SRC                           = 23,
	UnattachedWait_SRC                    = 24, 
  OrientedDebugAccessory_SRC            = 25,
  SRC_2_SNK_PR_SWAP_RD                  = 26
} TypeC_FSM_State_TypeDef;

/*0x12*/
/**
 * @brief USB PD Thermal_Fault
 * @Address 12h - Bit7
 * @Access RC
  */
typedef enum
{
  Junction_Temp_Below_145_degC      = 0,        /*!< DEFAULT */
  Junction_Temp_Above_145_degC      = 1
} Thermal_Fault_TypeDef;

/**
 * @brief USB PD VPU_OVP_Fault_Transition
 * @Address 12h - Bit5
 * @Access RC
  */
typedef enum
{
  No_VPU_OVP_Fault_Transition           = 0,        /*!< DEFAULT */
  VPU_OVP_Fault_Transition_Occurred     = 1
} VPU_OVP_Fault_Trans_TypeDef;

/**
 * @brief USB PD VPU_Presence_Transition
 * @Address 12h - Bit4
 * @Access RC
  */
typedef enum
{
  No_VPU_Presence_Transition           = 0,        /*!< DEFAULT */
  VPU_Presence_Transition_Occurred     = 1
} VPU_Presence_Trans_TypeDef;

/**
 * @brief USB PD VCONN_SW_RVP_FAULT_TRANS
 * @Address 12h - Bit2
 * @Access RC
  */
typedef enum
{
  No_VCONN_SW_RVP_Fault_Transition           = 0,        /*!< DEFAULT */
  VCONN_SW_RVP_Fault_Transition_Occurred     = 1
} VCONN_SW_RVP_Fault_Trans_TypeDef;

/**
 * @brief USB PD VCONN_SW_OCP_FAULT_TRANS
 * @Address 12h - Bit1
 * @Access RC
  */
typedef enum
{
  No_VCONN_SW_OCP_Fault_Transition           = 0,        /*!< DEFAULT */
  VCONN_SW_OCP_Fault_Transition_Occurred     = 1
} VCONN_SW_OCP_Fault_Trans_TypeDef;

/**
 * @brief USB PD VCONN_SW_OVP_FAULT_TRANS
 * @Address 12h - Bit0
 * @Access RC
  */
typedef enum
{
  No_VCONN_SW_OVP_Fault_Transition           = 0,        /*!< DEFAULT */
  VCONN_SW_OVP_Fault_Transition_Occurred     = 1
} VCONN_SW_OVP_Fault_Trans_TypeDef;

/*0x13*/
/**
 * @brief USB PD VPU_OVP_Fault
 * @Address 13h - Bit7
 * @Access RO
  */
typedef enum
{
  No_Over_Voltage_on_CC_pins            = 0,            /*!< DEFAULT: No over voltage on CC pins (safe condition) */
  Over_Voltage_on_CC_pins               = 1             /*!< Over voltage detected on CC pins */
} VPU_OVP_Fault_TypeDef;

/**
 * @brief USB PD VPU_Presence
 * @Address 13h - Bit6
 * @Access RO
  */
typedef enum
{
  PU_Voltage_on_CC_pins_below_UVLO              = 0,       /*!< Pull-up voltage on CC pins is below UVLO threshold  */
  PU_Voltage_on_CC_pins_above_UVLO              = 1        /*!< DEFAULT: Pull-up voltage on CC pins is above UVLO threshold (normal operating condition) */
} VPU_Presence_TypeDef;

/**
 * @brief USB PD VCONN_SW_RVP_FAULT_CC1
 * @Address 13h - Bit5
 * @Access RO
  */
typedef enum
{
  No_Reverse_Voltage_on_VCONN_Switch_to_CC1                 = 0,              /*!< DEFAULT: No reverse voltage on VCONN power switch connected to CC1  */
  Reverse_Voltage_on_VCONN_Switch_to_CC1                    = 1               /*!< Reverse voltage detected on VCONN power switch connected to CC1 */
} VCONN_SW_RVP_Fault_CC1_TypeDef;

/**
 * @brief USB PD VCONN_SW_RVP_FAULT_CC2
 * @Address 13h - Bit4
 * @Access RO
  */
typedef enum
{
  No_Reverse_Voltage_on_VCONN_Switch_to_CC2                 = 0,              /*!< DEFAULT: No reverse voltage on VCONN power switch connected to CC2 */
  Reverse_Voltage_on_VCONN_Switch_to_CC2                    = 1               /*!< Reverse voltage detected on VCONN power switch connected to CC2*/
} VCONN_SW_RVP_Fault_CC2_TypeDef;

/**
 * @brief USB PD VCONN_SW_OCP_FAULT_CC1
 * @Address 13h - Bit3
 * @Access RO
  */
typedef enum
{
  No_Current_Fault_on_VCONN_Switch_to_CC1                = 0,              /*!< DEFAULT: No short circuit or over current on VCONN power switch connected to CC1 */
  Current_Fault_on_VCONN_Switch_to_CC1                   = 1               /*!< Short circuit or over current detected on VCONN power switch connected to CC1 */
} VCONN_SW_OCP_Fault_CC1_TypeDef;

/**
 * @brief USB PD VCONN_SW_OCP_FAULT_CC2
 * @Address 13h - Bit2
 * @Access RO
  */
typedef enum
{
  No_Current_Fault_on_VCONN_Switch_to_CC2                = 0,              /*!< DEFAULT: No short circuit or over current on VCONN power switch connected to CC2 */
  Current_Fault_on_VCONN_Switch_to_CC2                   = 1               /*!< Short circuit or over current detected on VCONN power switch connected to CC2 */
} VCONN_SW_OCP_Fault_CC2_TypeDef;

/**
 * @brief USB PD VCONN_SW_OVP_FAULT_CC1
 * @Address 13h - Bit1
 * @Access RO
  */
typedef enum
{
  No_Over_Voltage_on_VCONN_Switch_to_CC1                = 0,              /*!< DEFAULT: No over voltage on VCONN power switch connected to CC1 */
  Over_Voltage_on_VCONN_Switch_to_CC1                   = 1               /*!< Over voltage detected on VCONN power switch connected to CC1 */
} VCONN_SW_OVP_Fault_CC1_TypeDef;

/**
 * @brief USB PD VCONN_SW_OVP_FAULT_CC2
 * @Address 13h - Bit0
 * @Access RO
  */
typedef enum
{
  No_Over_Voltage_on_VCONN_Switch_to_CC2                = 0,              /*!< DEFAULT: No over voltage on VCONN power switch connected to CC2 */
  Over_Voltage_on_VCONN_Switch_to_CC2                   = 1               /*!< Over voltage detected on VCONN power switch connected to CC2 */
} VCONN_SW_OVP_Fault_CC2_TypeDef;

/*0x17*/
/**
 * @brief Bus_Idle
 * @Address 17h - Bit3
 * @Access RC
  */
typedef enum
{
  Bus_is_idle                = 1,              /*!< DEFAULT: CC line bus is idle */
  Bus_is_busy                = 0               /*!< CC line bus is busy */
} Bus_Idle_TypeDef;

/*0x18*/
/**
 * @brief USB PD CC_CURRENT_ADVERTISED
 * @Address 18h - Bit7:6
 * @Access R/W
  */
typedef enum
{
  USB_C_Current_Default                 = 0,              /*!< DEFAULT: USB Current (500 mA or 900 mA) */
  USB_C_Current_1_5_A                   = 1,              /*!< 1.5 A USB Type-C Current */
  USB_C_Current_3_0_A                   = 2               /*!< 3.0 A USB Type-C Current */
} Current_Capability_Advertised_TypeDef;

/**
 * @brief USB PD SNK_DISCONNECT_MODE
 * @Address 18h - Bit5
 * @Access R/W
  */
typedef enum
{
  VBUS_or_SRC_removed                   = 0,              /*!< DEFAULT: exit from Attached.SNK to UnAttached.SNK is VBUS or SRC removed */
  VBUS_removed                          = 1               /*!< exit from Attached.SNK to UnAttached.SNK is VBUS removed */
} SNK_Disconnect_Mode_TypeDef;

/**
 * @brief USB PD CC_VCONN_DISCHARGE_EN
 * @Address 18h - Bit4
 * @Access R/W
  */
typedef enum
{
  VCONN_Discharge_Disable_on_CC_pin             = 0,              /*!< DEFAULT: VCONN discharge disabled on CC pin */
  VCONN_Discharge_Enable_250ms_on_CC_pin        = 1               /*!< VCONN discharge enabled for 250 ms on CC pin */
} VCONN_Discharge_Status_TypeDef;

/**
 * @brief USB PD DR_SWAP_EN
 * @Address 18h - Bit3
 * @Access R/W
  */
typedef enum
{
  Data_Role_Swap_Disable                = 0,              /*!< DEFAULT: Data role swap capability is disabled; */
  Data_Role_Swap_Enable                 = 1               /*!< Data role swap capability is enabled. */
} Data_Role_Swap_TypeDef;

/**
 * @brief USB PD PR_SWAP_EN
 * @Address 18h - Bit2
 * @Access R/W
  */
typedef enum
{
  Power_Role_Swap_Disable                = 0,              /*!< DEFAULT: Power role swap capability is disabled; */
  Power_Role_Swap_Enable                 = 1               /*!< Power role swap capability is enabled. */
} Power_Role_Swap_TypeDef;

/**
 * @brief USB PD VCONN_SWAP_EN
 * @Address 18h - Bit1
 * @Access R/W
  */
typedef enum
{
  VCONN_Role_Swap_Disable                = 0,              /*!< DEFAULT: VCONN swap capability is disabled; */
  VCONN_Role_Swap_Enable                 = 1               /*!< VCONN swap capability is enabled. */
} VCONN_Role_Swap_TypeDef;

/**
 * @brief USB PD CC_VCONN_SUPPLY_EN
 * @Address 18h - Bit0
 * @Access R/W
  */
typedef enum
{
  VCONN_Supply_Capability_Disable_on_CC_pin          = 0,              /*!< VCONN supply capability disabled on CC pin */
  VCONN_Supply_Capability_Enable_on_CC_pin           = 1               /*!< DEFAULT: VCONN supply capability enabled on CC pin */
} VCONN_Supply_Status_TypeDef;

/*0x1E*/
/**
 * @brief USB PD CC_VCONN_SWITCH_CTRL
 * @Address 1Eh - Bit3:0
 * @Access R/W
  */
typedef enum
{
  ILIM_350_ma           = 0,                            /*!< DEFAULT */
  ILIM_300_ma           = 1,
  ILIM_250_ma           = 2,
  ILIM_200_ma           = 3,
  ILIM_150_ma           = 4,
  ILIM_100_ma           = 5,
  ILIM_400_ma           = 6,
  ILIM_450_ma           = 7,
  ILIM_500_ma           = 8,
  ILIM_550_ma           = 9,
  ILIM_600_ma           = 10
} VCONN_Switch_Current_Limit_TypeDef;

/*0x1F*/
/**
 * @brief USB PD TYPE-C_CTRL
 * @Address 1Fh - Bit7:4
 * @Access R/W
  */
typedef enum
{
  NO_REQ                                        = 0,            /*!< DEFAULT */
  PD_HARD_RESET_COMPLETE_REQ                    = 1,
  PD_HARD_RESET_TURN_OFF_VCONN_REQ              = 2,
  PD_HARD_RESET_PORT_CHANGE_2_DFP_REQ           = 3,
  PD_HARD_RESET_PORT_CHANGE_2_UFP_REQ           = 4,
  PD_PR_SWAP_SNK_VBUS_OFF_REQ                   = 5,
  PD_PR_SWAP_SRC_VBUS_OFF_REQ                   = 6,
  PD_PR_SWAP_RP_ASSERT_REQ                      = 7,
  PD_PR_SWAP_RD_ASSERT_REQ                      = 8,
  PD_DR_SWAP_PORT_CHANGE_2_DFP_REQ              = 9,
  PD_DR_SWAP_PORT_CHANGE_2_UFP_REQ              = 10,
  PD_VCONN_SWAP_TURN_ON_VCONN_REQ               = 11,
  PD_VCONN_SWAP_TURN_OFF_VCONN_REQ              = 12,
	PD_PR_SWAP_PS_RDY_REQ                         = 13,
  PD_HARD_RESET_RECEIVED_REQ                    = 14,
  PD_HARD_RESET_SEND_REQ                        = 15
} Type_C_CTRL_TypeDef;

/*0x20*/
/**
 * @brief USB PD VCONN_MONITORING_EN
 * @Address 20h - Bit7
 * @Access R/W
  */
typedef enum
{
  Disable_UVLO_thr_detect_on_VCONN      = 0,              /*!< Disable UVLO threshold detection on VCONN pin */
  Enable_UVLO_thr_detect_on_VCONN       = 1               /*!< DEFAULT: Enable UVLO threshold detection on VCONN pin */
} VCONN_Monitoring_TypeDef;

/**
 * @brief USB PD VCONN_UVLO_THRESHOLD
 * @Address 20h - Bit6
 * @Access R/W
  */
typedef enum
{
  Hi_UVLO_thr_of_4_65_V                 = 0,              /*!< DEFAULT: High UVLO threshold of 4.65 V */
  Lo_UVLO_thr_of_2_65_V                 = 1               /*!< Low UVLO threshold of 2.65 V (case of VCONN-powered accessories operating down to 2.7 V) */
} VCONN_UVLO_Threshold_TypeDef;

/*0x21*/
/* VBUS_ PDO_MONITORING_CTRL / VBUS Select */

/*0x22*/
/* VBUS_RANGE_MONITORING_CTRL */

/*0x23*/
/**
 * @brief USB PD SW RESET
 * @Address 23h - Bit0
 * @Access R/W
  */
typedef enum
{
  No_SW_RST                             = 0,           /*!< DEFAULT: Device reset is performed through the hardware RESET pin */
  SW_RST                                = 1            /*!< Force the device reset as long as this bit value is set */
} SW_RESET_TypeDef;

/*0x24*/
/**
 * @brief USB PD PWR_ACC_TRY_SNK_EN
 * @Address 24h - Bit1
 * @Access R/W
  */
typedef enum
{
  Pwr_Acc_Try_SNK_Disable               = 0,           /*!< DEFAULT: Disable transition from Powered.Accessory state to Try.SNK state */
  Pwr_Acc_Try_SNK_Enable                = 1            /*!< Enable transition from Powered.Accessory state to Try.SNK state */
} Pwr_Acc_Try_SNK_TypeDef;

/**
 * @brief USB PD PWR_ACC_DETECT_EN
 * @Address 24h - Bit0
 * @Access R/W
  */
typedef enum
{
  Pwr_Acc_Detect_Disable               = 0,           /*!< Disable the powered accessory detection */
  Pwr_Acc_Detect_Enable                = 1            /*!< DEFAULT: Enable the powered accessory detection */
} Pwr_Acc_Detect_TypeDef;

/*0x25*/
/* VBUS_DISCHARGE_TIME_CTRL */

/*0x26*/
/**
 * @brief USB PD VBUS_DISCHARGE_CTRL
 * @Address 26h - Bit7
 * @Access R/W
  */
typedef enum
{
  VBUS_Discharge_Path_Disable                  = 0,              /*!<  Disable the VBUS discharge path */
  VBUS_Discharge_Path_Enable                   = 1               /*!<  DEFAULT: Enable the VBUS discharge path */
} VBUS_Discharge_State_TypeDef;

/*0x27*/
/**
 * @brief USB PD VBUS_SINK_EN
 * @Address 27h - Bit1
 * @Access R/W
  */
typedef enum
{
  VBUS_SNK_Pwr_Disable                  = 0,              /*!< DEFAULT: BUS Sink power path disabled */
  VBUS_SNK_Pwr_Enable                   = 1               /*!< VBUS Sink power path enabled */
} VBUS_SNK_State_TypeDef;

/**
 * @brief USB PD VBUS_SOURCE_EN
 * @Address 27h - Bit0
 * @Access R/W
  */
typedef enum
{
  VBUS_SRC_Pwr_Disable                  = 0,              /*!< DEFAULT: VBUS Source power path disabled */
  VBUS_SRC_Pwr_Enable                   = 1               /*!< VBUS Source power path enabled */
} VBUS_SRC_State_TypeDef;

/**
 * @brief USB PD POWER_MODE
 * @Address 28h - Bit2:0
 * @Access R/W
  */
typedef enum
{
  SRC_with_accessory_supp               = 0,            /*!< Source power role with accessory support */
  SNK_with_accessory_supp               = 1,            /*!< Sink power role with accessory support */
  SNK_without_accessory_supp            = 2,            /*!< Sink power role without accessory support */
  DRP_w_accessory_supp                  = 3,            /*!< DEFAULT: Dual power role with accessory support */
  DRP_w_accessory_TrySRC_supp           = 4,            /*!< Dual role power with accessory and Try.SRC support */
  DRP_w_accessory_TrySNK_supp           = 5             /*!< Dual role power  with accessory and Try.SNK support */
} Power_Mode_TypeDef;

/*0x2E*/
/**
 * @brief USB PD VDD_OVLO_DISABLE
 * @Address 2Eh - Bit6
 * @Access R/W
  */
typedef enum
{
  VDD_OVLO_Enable                       = 0,              /*!< DEFAULT: Enable OVLO threshold detection on VDD pin */
  VDD_OVLO_Disable                      = 1               /*!< Disable OVLO threshold detection on VDD pin */
} VDD_OVLO_Threshold_TypeDef;

/**
 * @brief USB PD VBUS_RANGE_DISABLE
 * @Address 2Eh - Bit4
 * @Access R/W
  */
typedef enum
{
  VBUS_Range_Enable                       = 0,              /*!< DEFAULT: Enable VBUS voltage range detection */
  VBUS_Range_Disable                      = 1               /*!< Disable VBUS voltage range detection (UVLO detection instead) */
} VBUS_Range_State_TypeDef;

/**
 * @brief USB PD VBUS_VSAFE0V_THRESHOLD
 * @Address 2Eh - Bit2:1
 * @Access R/W
  */
typedef enum
{
  VBUS_vSafe0V_Thr_0_6V                 = 0,              /*!< DEFAULT: VBUS vSafe0V threshold = 0.6 V */
  VBUS_vSafe0V_Thr_0_9V                 = 1,              /*!< VBUS vSafe0V threshold = 0.9 V */
  VBUS_vSafe0V_Thr_1_2V                 = 2,              /*!< VBUS vSafe0V threshold = 1.2 V */
  VBUS_vSafe0V_Thr_1_8V                 = 3               /*!< VBUS vSafe0V threshold = 1.8 V */
} VBUS_VSAFE0V_Threshold_TypeDef;

/**
 * @brief USB PD VDD_UVLO_DISABLE
 * @Address 2Eh - Bit0
 * @Access R/W
  */
typedef enum
{
  VDD_UVLO_Enable                       = 0,              /*!< Enable UVLO threshold detection on VDD pin */
  VDD_UVLO_Disable                      = 1               /*!< DEFAULT:Disable UVLO threshold detection on VDD pin */
} VDD_UVLO_Threshold_TypeDef;

/**
 * @brief  	STUSB1602 DEVICE CUT register Structure definition
 * @Address 	2Fh
 * @Access 	RO
 * @details 	This register provides information on current status of the VBUS power path activation
  *		through VBUS_EN_SRC pin in Source power role and VBUS_EN_SNK pin in Sink power role.
 */
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t Reserved_0_2		        :	2;
        uint8_t DEVICE_CUT		        :	3;
        uint8_t Reserved_5_6		        :	2;
        uint8_t WO_VCONN                        :       1;
  } b;
} STUSB1602_DEVICE_CUT_RegTypeDef ;

/**
 * @brief DEVICE_CUT
 * @Address 2Fh - Bit4:2
 * @Access R/W
  */
typedef enum
{
  reserved0                       = 0,              /*!< too old cut of 1602 */
  reserved1                       = 1,              /*!< too old cut of 1602 */
  reserved2                       = 2,              /*!< too old cut of 1602 */
  Cut_1                           = 3,              /*!< cut 1.2 of 1602 */
  Cut_1_A                         = 4,              /*!< cut 1.3 of 1602 */
} DEVICE_CUT_TypeDef;

/**
 * @brief DEVICE_CUT
 * @Address 2Fh - Bit1:0
 * @Access R/W
  */
typedef enum
{
  reserved_0                       = 0,              /*reserved */
  reserved_1                       = 1,              /*reserved */
  NVM_OK                           = 2,              /*NVM loaded */
  reserved_2                       = 3,              /*reserved */
} NVM_OK_TypeDef;

/**
 * @}
 */

/**
 * @}
 */

/* Exported functions --------------------------------------------------------*/
/** @defgroup USBPD_DEVICE_STUSB1602_LIBRARY_Exported_Prototypes USBPD DEVICE STUSB1602 LIBRARY Exported functions prototypes
* @{
*/
void STUSB1602_Driver_Init(uint8_t PortNum, I2C_HandleTypeDef I2CxHandle);
STUSB1602_StatusTypeDef STUSB1602_ReadReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
uint8_t STUSB1602_ReadRegSingle(uint8_t Addr, uint8_t Reg);
STUSB1602_StatusTypeDef STUSB1602_WriteReg(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
STUSB1602_StatusTypeDef STUSB1602_WriteRegSingle(const uint8_t Value, uint8_t Addr, uint8_t Reg);
#ifdef __VVAR
STUSB1602_StatusTypeDef STUSB1602_WriteReg_P1(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
STUSB1602_StatusTypeDef STUSB1602_ReadReg_P1(uint8_t* pBuffer, uint8_t Addr, uint8_t Reg, uint16_t Size);
#endif

/* 0x0B -> 0x10 */
STUSB1602_ALERT_MONITORING_TypeDef STUSB1602_Alert_Monitoring_Get(uint8_t Addr);
/* 0x0B -> 0x0C */
STUSB1602_ALERT_STATUS_RegTypeDef STUSB1602_Alert_Raise_Get(uint8_t Addr);

/*0x0B*/
STUSB1602_ALERT_STATUS_RegTypeDef STUSB1602_Alert_Get(uint8_t Addr);

/*0x0C*/
CC_Detect_Alrt_Int_Mask_Status_TypeDef STUSB1602_CC_Detect_Alrt_Int_Mask_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_CC_Detect_Alrt_Int_Mask_Set(uint8_t Addr, CC_Detect_Alrt_Int_Mask_Status_TypeDef st);
Monitor_Alrt_Int_Mask_Status_TypeDef STUSB1602_Monitoring_Status_Alrt_Int_Mask_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Monitoring_Status_Alrt_Int_Mask_Set(uint8_t Addr, Monitor_Alrt_Int_Mask_Status_TypeDef st);
HW_Fault_Alrt_Int_Mask_Status_TypeDef STUSB1602_HW_Fault_Alrt_Int_Mask_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_HW_Fault_Alrt_Int_Mask_Set(uint8_t Addr, HW_Fault_Alrt_Int_Mask_Status_TypeDef st);

/*0x0D*/
Attach_State_Trans_TypeDef STUSB1602_Attach_State_Trans_Get(uint8_t Addr);

/*0x0E*/
STUSB1602_MONITORING_STATUS_RegTypeDef STUSB1602_Monitoring_Status_Reg_Get(uint8_t Addr);
Attach_State_TypeDef STUSB1602_Attach_State_Get(uint8_t Addr);
VCONN_Supply_State_TypeDef STUSB1602_VCONN_Supply_State_Get(uint8_t Addr);
Data_Role_TypeDef STUSB1602_Data_Role_Get(uint8_t Addr);
Power_Role_TypeDef STUSB1602_Power_Role_Get(uint8_t Addr);
StartUp_Mode_TypeDef STUSB1602_StartUp_Mode_Get(uint8_t Addr);
Attach_Mode_TypeDef STUSB1602_Attach_Mode_Get(uint8_t Addr);

/*0x0F*/
STUSB1602_MONITORING_STATUS_TRANS_RegTypeDef STUSB1602_Monitoring_Status_Trans_Reg_Get(uint8_t Addr);
PD_TypeC_Handshake_TypeDef STUSB1602_PD_TypeC_HandShake_Get(uint8_t Addr);
VBUS_Valid_Trans_TypeDef STUSB1602_VBUS_Valid_Trans_Get(uint8_t Addr);
VBUS_VSAFE0V_Trans_TypeDef STUSB1602_VBUS_VSAFE0V_Trans_Get(uint8_t Addr);
VBUS_Presence_Trans_TypeDef STUSB1602_VBUS_Presence_Trans_Get(uint8_t Addr);
VCONN_Presence_Trans_TypeDef STUSB1602_VCONN_Presence_Trans_Get(uint8_t Addr);

/*0x10*/
VBUS_Valid_TypeDef STUSB1602_VBUS_Valid_Get(uint8_t Addr);
VBUS_VSAFE0V_TypeDef STUSB1602_VBUS_VSAFE0V_Get(uint8_t Addr);
VBUS_Presence_TypeDef STUSB1602_VBUS_Presence_Get(uint8_t Addr);
VCONN_Presence_TypeDef STUSB1602_VCONN_Presence_Get(uint8_t Addr);

/*0x11*/
CCxPin_Attached_TypeDef STUSB1602_CCx_Pin_Attach_Get(uint8_t Addr);
Sink_Power_State_TypeDef STUSB1602_Sink_Power_State_Get(uint8_t Addr);
TypeC_FSM_State_TypeDef STUSB1602_TypeC_FSM_State_Get(uint8_t Addr);

/*0x12*/
STUSB1602_HW_FAULT_STATUS_TRANS_RegTypeDef STUSB1602_Hard_Fault_Trans_Status_Get(uint8_t Addr);
Thermal_Fault_TypeDef STUSB1602_Thermal_Fault_Get(uint8_t Addr);
VPU_OVP_Fault_Trans_TypeDef STUSB1602_VPU_OVP_Fault_Trans_Get(uint8_t Addr);
VPU_Presence_Trans_TypeDef STUSB1602_VPU_Presence_Trans_Get(uint8_t Addr);
VCONN_SW_RVP_Fault_Trans_TypeDef STUSB1602_VCONN_SW_RVP_Fault_Trans_Get(uint8_t Addr);
VCONN_SW_OCP_Fault_Trans_TypeDef STUSB1602_VCONN_SW_OCP_Fault_Trans_Get(uint8_t Addr);
VCONN_SW_OVP_Fault_Trans_TypeDef STUSB1602_VCONN_SW_OVP_Fault_Trans_Get(uint8_t Addr);

/*0x13*/
VPU_OVP_Fault_TypeDef STUSB1602_VPU_OVP_Fault_Get(uint8_t Addr);
VPU_Presence_TypeDef STUSB1602_VPU_Presence_Get(uint8_t Addr);
VCONN_SW_RVP_Fault_CC1_TypeDef STUSB1602_VCONN_SW_RVP_Fault_CC1_Get(uint8_t Addr);
VCONN_SW_RVP_Fault_CC2_TypeDef STUSB1602_VCONN_SW_RVP_Fault_CC2_Get(uint8_t Addr);
VCONN_SW_OCP_Fault_CC1_TypeDef STUSB1602_VCONN_SW_OCP_Fault_CC1_Get(uint8_t Addr);
VCONN_SW_OCP_Fault_CC2_TypeDef STUSB1602_VCONN_SW_OCP_Fault_CC2_Get(uint8_t Addr);
VCONN_SW_OVP_Fault_CC1_TypeDef STUSB1602_VCONN_SW_OVP_Fault_CC1_Get(uint8_t Addr);
VCONN_SW_OVP_Fault_CC2_TypeDef STUSB1602_VCONN_SW_OVP_Fault_CC2_Get(uint8_t Addr);

/*0x17*/
Bus_Idle_TypeDef STUSB1602_Bus_Idle_Status_Get(uint8_t Addr);

/*0x18*/
STUSB1602_CC_DETECTION_STATUS_RegTypeDef STUSB1602_CC_Detection_Status_Get(uint8_t Addr);
Current_Capability_Advertised_TypeDef STUSB1602_Current_Advertised_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Current_Advertised_Set(uint8_t Addr, Current_Capability_Advertised_TypeDef curr_cap);
SNK_Disconnect_Mode_TypeDef STUSB1602_SNK_Disconnect_Mode_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_SNK_Disconnect_Mode_Status_Set(uint8_t Addr, SNK_Disconnect_Mode_TypeDef st);
VCONN_Discharge_Status_TypeDef STUSB1602_VCONN_Discharge_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VCONN_Discharge_Status_Set(uint8_t Addr, VCONN_Discharge_Status_TypeDef st);
Data_Role_Swap_TypeDef STUSB1602_Data_Role_Swap_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Data_Role_Swap_Status_Set(uint8_t Addr, Data_Role_Swap_TypeDef st);
Power_Role_Swap_TypeDef STUSB1602_Power_Role_Swap_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Power_Role_Swap_Status_Set(uint8_t Addr, Power_Role_Swap_TypeDef st);
VCONN_Role_Swap_TypeDef STUSB1602_VCONN_Role_Swap_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VCONN_Role_Swap_Status_Set(uint8_t Addr, VCONN_Role_Swap_TypeDef st);
VCONN_Supply_Status_TypeDef STUSB1602_VCONN_Supply_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VCONN_Supply_Status_Set(uint8_t Addr, VCONN_Supply_Status_TypeDef st);

/*0x1E*/
VCONN_Switch_Current_Limit_TypeDef STUSB1602_VCONN_Switch_Current_Limit_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VCONN_Switch_Current_Limit_Set(uint8_t Addr, VCONN_Switch_Current_Limit_TypeDef curr_lim);

/*0x1F*/
Type_C_CTRL_TypeDef STUSB1602_Type_C_Control_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Type_C_Control_Set(uint8_t Addr, Type_C_CTRL_TypeDef Ctrl);
Power_Mode_TypeDef STUSB1602_Power_Mode_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Power_Mode_Set(uint8_t Addr, Power_Mode_TypeDef Pwr);

/*0x20*/
VCONN_Monitoring_TypeDef STUSB1602_VCONN_Monitor_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VCONN_Monitor_Status_Set(uint8_t Addr, VCONN_Monitoring_TypeDef st);
VCONN_UVLO_Threshold_TypeDef STUSB1602_VCONN_UVLO_Thresh_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VCONN_UVLO_Thresh_Status_Set(uint8_t Addr, VCONN_UVLO_Threshold_TypeDef thr);

/*0x21*/
uint16_t STUSB1602_VBUS_Select_Status_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_Select_Status_Set(uint8_t Addr, uint16_t mV);

/*0x22*/
uint8_t STUSB1602_VBUS_VShift_High_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_VShift_High_Set(uint8_t Addr, uint8_t Set);
int8_t STUSB1602_VBUS_VShift_Low_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_VShift_Low_Set(uint8_t Addr, int8_t Set);

/*0x23*/
SW_RESET_TypeDef STUSB1602_SW_RESET_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_SW_RESET_Set(uint8_t Addr, SW_RESET_TypeDef Rst);

/*0x24*/
Pwr_Acc_Try_SNK_TypeDef STUSB1602_Pwr_Acc_Try_SNK_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Pwr_Acc_Try_SNK_Set(uint8_t Addr, Pwr_Acc_Try_SNK_TypeDef st);
Pwr_Acc_Detect_TypeDef STUSB1602_Pwr_Acc_Detect_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_Pwr_Acc_Detect_Set(uint8_t Addr, Pwr_Acc_Detect_TypeDef st);

/*0x25*/
uint16_t STUSB1602_VBUS_Discharge_Time_to_0V_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_Discharge_Time_to_0V_Set(uint8_t Addr, uint16_t tim);
uint16_t STUSB1602_VBUS_Discharge_Time_to_PDO_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_Discharge_Time_to_PDO_Set(uint8_t Addr, uint16_t tim);

/*0x26*/
VBUS_Discharge_State_TypeDef STUSB1602_VBUS_Discharge_State_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_Discharge_State_Set(uint8_t Addr, VBUS_Discharge_State_TypeDef st);

/*0x27*/
VBUS_SNK_State_TypeDef STUSB1602_VBUS_SNK_State_Get(uint8_t Addr);
VBUS_SRC_State_TypeDef STUSB1602_VBUS_SRC_State_Get(uint8_t Addr);

/*0x2E*/
VDD_OVLO_Threshold_TypeDef STUSB1602_VDD_OVLO_Threshold_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VDD_OVLO_Threshold_Set(uint8_t Addr, VDD_OVLO_Threshold_TypeDef st);
VBUS_Range_State_TypeDef STUSB1602_VBUS_Range_State_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_Range_State_Set(uint8_t Addr, VBUS_Range_State_TypeDef st);
VBUS_VSAFE0V_Threshold_TypeDef STUSB1602_VBUS_VSAFE0V_Threshold_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VBUS_VSAFE0V_Threshold_Set(uint8_t Addr, VBUS_VSAFE0V_Threshold_TypeDef st);
VDD_UVLO_Threshold_TypeDef STUSB1602_VDD_UVLO_Threshold_Get(uint8_t Addr);
STUSB1602_StatusTypeDef STUSB1602_VDD_UVLO_Threshold_Set(uint8_t Addr, VDD_UVLO_Threshold_TypeDef st);
DEVICE_CUT_TypeDef STUSB1602_DEVICE_CUT_Get(uint8_t Addr);
NVM_OK_TypeDef STUSB1602_NVM_OK_Get(uint8_t Addr);

STUSB1602_StatusTypeDef STUSB1602_Type_C_Command(uint8_t Addr, Type_C_CTRL_TypeDef Ctrl);
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

#ifdef __cplusplus
}
#endif

#endif /* __STUSB1602_DRIVER_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

