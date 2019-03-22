# X-CUBE-USB-PD
"X-Cube-USB-PD" is the open source Firmware for STM32Fx microcontrollers.  It support USB Type-C and USB PD up to 100W. <br>

* The FW can be tested with the following Evaluation Kit:
  * P-NUCLEO-USB002 (based on STUSB1602 controller + STM32F0)
  * P-NUCLEO-USB001 (based on Discrete components + STM32F0)
  * STM32G071B-DISCO discovery kit (based on STM32G0)
  * STM32G081B-EVAL Evaluation board (contains 2 USB-C ports)
  
&nbsp;&nbsp; (The FW sources are located in the folder: STM32CubeExpansion_USBPD_F0)

Purpose:
--------
USB type-C (USB-C) and USB Power Delivery (USB-PD) controller based on STM32.  <br>

Work in any mode: DFP, UFP, and DRP (Dual Role Power) <br>
Specifiation: Compliant with USB PD v2.0 & v3.0, and USB Type-C v1.2

Description:
--------
This is the firmware to use with either :
1. STM32 and a dedicated USB PD controller  
2. or STM32 and GPIO/ADC pins to simulate a USB Type-C + PD controller.

- Case 1: Optimised PCB solution. No external components needed.
- Case 2: No specific USB-PD IC is required. Everything is managed by software. Making this discrete solution a low cost USB-PD controller. Just few external component are required like resitors, capacitors, and Mosfets.

These solutions from the silicon provider (STMicroelectronics) has been certified by the official USB organisation (USB-IF: USB implementer forum).
It only works on STM32 microcontroller (ARM Cortex-M based). But may be ported on other devices.

Hardware:
--------
Several boards are available to implement and test the USB-C PD firmware.   <br>

* Provider: STMicroelectronics  

* Development board: __P-NUCLEO-USB002__ &nbsp; &nbsp; (contains NUCLEO-F072RB Board + MB1303 expansion board)  <br>   <br>
Onboard Devices:
  * Embedded Controller: STM32F072  (runs the USB-PD stack in software)
  * Analog Front End: STUSB1602 USB-PD controller (high voltage tolerant 28V)   

    * Note:  NUCLEO-F072RB is the Nucleo board.       <br>
MB1303 is the Xpansion board containing STUSB1602, to connect on top of the Nucleo board.

<br>
   
* Development board: __P-NUCLEO-USB001__ &nbsp; &nbsp; (contains NUCLEO-F072RB Board + MB1257 expansion board)  <br>   <br>
Onboard Devices:
  * Embedded Controller: STM32F072  (runs the USB-PD stack in software)
  * Analog Front End: Discrete implementation with several external components
    * warning: STM32F0 is not 5V compatible, but 3.6V max --> needs external components.

* Development board: __STM32G071B-DISCO__ &nbsp; &nbsp; (USB-C Discovery kit for STM32G071RB MCU)  <br>   <br>
Onboard Devices:
  * Embedded Controller: STM32G071 with UCPD feature (runs the USB-PD stack in software)
  * Analog Front End: Discrete implementation  (warning: STM32G0 is not 5V compatible --> needs external components).
    * warning: STM32G0 is not 5V tolerant, but 3.6V max --> needs external components.
    
* Development board: __STM32G081B-EVAL__ Evaluation board &nbsp; &nbsp; (the daughterboard features 2 independent USB-C ports controlled)  <br>   <br>
Onboard Devices:
  * Embedded Controller: STM32G081RBT6 with UCPD feature (runs the USB-PD stack in software)
  * Analog Front End: Discrete implementation
    * warning: STM32G0 is not 5V tolerant, but 3.6V max --> needs external components.
    



--------
X Cube USB PD  
STUSB1602 solution or Discrete solution
STM32G0 world 1st general purpose MCU with USB-C support
