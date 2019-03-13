# X-CUBE-USB-PD
"X-Cube-USB-PD" is the Firmware for STM32F0 Nucleo board.  It support USB Type-C and USB PD up to 100W <br>

* FW to be used with the following Evaluation Kit:
  * P-NUCLEO-USB002 (based on STUSB1602 controller)
  * or P-NUCLEO-USB001 (based on Discrete components)

&nbsp;&nbsp; (The FW is located in the folder: STM32CubeExpansion_USBPD_F0)

Purpose:
--------
USB type-C (USB-C) and USB Power Delivery (USB-PD) controller based on STM32.  <br>

Work in any mode: DFP, UFP, and DRP (Dual Role Power) <br>
Compliant with USB PD v2.0 & v3.0

Description:
--------
This is the firmware using STM32 and GPIO pins to behave as a USB Type-C + PD controller.

No specific USB-PD IC is required. Everything is managed by software. Making this discrete solution a low cost USB-PD controller.
Just few external component are required like resitors, capacitors, and Mosfets.

This solution from the silicon provider (STMicro) has been certified by the official USB organisation (USB-IF: USB implementer forum).
It only works on STM32 microcontroller (ARM Cortex-M based).

Hardware:
--------
Provider: STMicroelectronics  

* Development board: P-NUCLEO-USB002 &nbsp; &nbsp; (contains NUCLEO-F072RB Board + MB1303 expansion board)  <br>
* Onboard Devices : 
  * Embedded Controller: STM32F072  (runs the USB-PD stack)
  * Analog Front End: STUSB1602 USB-PD controller (high voltage tolerant 28V)   

    * Note:  NUCLEO-F072RB is the Nucleo board.   
MB1303 is the Xpansion board containing STUSB1602, to connect on top of the Nucleo board.

<br>
   
* Development board: P-NUCLEO-USB001 &nbsp; &nbsp; (contains NUCLEO-F072RB Board + MB1257 board)   
* Onboard Devices : 
  * Embedded Controller: STM32F072  (runs the USB-PD stack)
  * Analog Front End: Discrete implementation with several external components   


--------
X Cube USB PD  
STUSB1602 solution or Discrete solution
