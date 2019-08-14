# 2019 evaluation for the infantry robot
## Chassis
### All-direction driving
* ME: use four 3508 motors and connect them to four McNamee universal wheels. (forms the sharp of an 'X' from the above)
* EE: copy the control logic from the official code, but tune the PID on our own. (pls refer to the code for details)
### Power management
* EE:
** software**: make use of a current sensor and a voltage sensor to detect the real-time power and turn on the supercapacitor when power is byond the limitation. (pls refer to the code for details)
** hardware**: below sees the schematic
### Dodging (rotating for 360 degree)
* ME: 
1. use 6010 motor from Dji as the yaw motor that connects the gimbal and chassis.
2. clear all the components that stands in the way of a rotating gimbal.
* EE: 
** hardware**: conductive slip ring (PSR-TS) is used and is fitted in the 6010 motor.

** software**:  fix `vw` in `prc_info` when `v` on the keyboard is pressed (pls refer to the code for details)
### Sudden accelerating and slowing
* EE: limit the output of PID when `Shift` or `Ctrl` is pressed (pls refer to the code for details)
## Gimbal
### Normal function like turn left/right, pitch up/down
*ME: one pitch and one yaw motor (both 6010)
*EE: PID takes position feedback from imu (not from encoder)
### shooting
*ME: two snail motors with customized rubber ring(friction wheel)and one 6030 motor(trigger motor) form the shooting mechanism

**[ATTENTION]** big chance of bullet blocking in most of the former design(**NO blocking** should be the highest priority)
*EE: send PWM signal to the esc of the snail motors to make it rotate (never stop in real match)
### Single/Continuous shooting
*EE: set the reference of the trigger motor to 45 degree or 360 degree (pls refer to the code for details)
### Auto-aiming
