# X-CUBE-USB-PD
Firmware X-Cube-USB-PD for STM32F0 Nucleo board  
-> to be used with Evaluation Kit named : P-NUCLEO-USB002 (based on STUSB1602 controller), or P-NUCLEO-USB001 (Discrete)   

&nbsp;&nbsp; (folder: STM32CubeExpansion_USBPD_F0)

Purpose:
--------
USB type-C (USB-C) and USB Power Delivery (USB-PD) controller based on STM32

Description:
--------
This is the firmware using STM32 and GPIO pins to behave as a USB Type-C + PD controller.

No specific USB-PD IC is required. Everything is managed by software. Making this discrete solution a low cost USB-PD controller.
Just few external component are required like resitors, capacitors, and Mosfets.

This solution from the silicon provider (STMicro) has been certified by the official USB organisation (USB-IF: USB implementer forum).
It only works on STM32 microcontroller (ARM Cortex-M based).

Hardware:
--------
* Development board: P-NUCLEO-USB002 &nbsp; &nbsp; (contains NUCLEO-F072RB Board + MB1303 board)   
Devices : 
  * Embedded Controller: STM32F072  (runs the USB-PD stack)
  * Analog Front End: STUSB1602 controller (high voltage tolerant 28V)   

Provider: STMicroelectronics   

Note:  NUCLEO-F072RB is the Nucleo board.   
MB1303 is the Xpansion board containing STUSB1602.   

<br>
   
* Development board: P-NUCLEO-USB001  
Devices : 
  * Embedded Controller: STM32F072  (runs the USB-PD stack)
  * Analog Front End: Discrete implementation with several external components   

Provider: STMicroelectronics

--------
X Cube USB PD  
Discrete or STUSB1602 solution
