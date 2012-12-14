#CNC Control Panel: Interface MCU firmware#


(C) 2012 Radu - Eosif Mihailescu <radu.mihailescu@linux360.ro>

Hereby licensed under the **GNU Generic Public License v3**, a copy of whose text
can be accessed [here](http://www.gnu.org/licenses/gpl.html).

This repository contains the source of the firmware that goes on the *Interface MCU*
in the *CNC Control Panel* project.

The current job of the Interface MCU is controlling the crossbar switch; 
generating the SPINDLE, COOL and SPINDLE_PWM signals and E-Stop detection.

##WARNING##
*Unless and until explicitly noted otherwise, this code is laboratory quality and
not intended for usage in connection with actual moving parts such as CNC spindles.
Should you choose to test or use this software in a live environment, you do so
entirely at your own risk.*