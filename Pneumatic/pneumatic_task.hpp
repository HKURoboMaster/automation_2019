#ifndef __PNEU_T__
#define __PNEU_T__


//Board Type definination
//#define ARDUINO_MEGA
//#define ARDUINO_DUE
//System bags
//#include <Thread.h>
//#include <ThreadController.h>
#ifdef ARDUINO_MEGA
#include <MsTimer2.h>
#endif
#ifdef ARDUINO_DUE
#include <DueTimer.h>
#endif
#include <Wire.h>

#include "sensors.h"
#include "actions.h"
#include "cylinders.h"
#include "Arduino.h"



//Parameter definination
//PINS definination
#define RODLESS_LEFT    34
#define RODLESS_RIGHT   35
#define EJECT_LEFT      36
#define EJECT_RIGHT     37
#define GRABBER         38
#define ARM             39
//Sensor_pins definination
#define SENSOR1         22
#define SENSOR2         23
#define SENSOR3         24
#define SENSOR4         25
#define SENSOR5         26

#define NULL_PIN        1024
// Sensor Update parameter
#define SHIFTER         21
#define UPDATE_FRQ      2
#define ACTIVATE        1
#define DEACTIVATE      0
//Cylinders Working states
#define ROD_SET         1
#define ROD_RESET       0
#define ROD_STOP        2
#define EJECT_RESET     1
#define EJECT_SET           0
#define GRABBER_GRAB    1
#define GRABBER_RELEASE 0
#define ARM_REACH       1
#define ARM_RETRACT     0

//Command
#define TASK1           0
#define TASK2           1
#define TASK3           2
#define RESET_TASK      3
#define EJECT_TASK      4
#define NULL_TASK       1024
// Communication related
#define DEVICE_ID       0x10u //16 in decimal
#define CLOCK_FRQ       100000 
//init Parameter define



// Cylinder class instance declaration




#endif 