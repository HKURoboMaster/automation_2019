#ifndef __SENSORS__
#define __SENSORS__

#include "Arduino.h"

// V1.0 Regard Sensor as On/Off Switch
 // Collect Three Boxes
//bool sensor_array2[2] = {0,0};   // Collect Two Boxes
 //


void sensors_Init();                    //Pins Initialize sensors
void TIM_IRQ_Init(); // Init Or update Timer interrupts according to active sensors
void sensors_update();        // Call Back function from timer interrupt to update the sensor information
void TIM_Stop();     //Which ever being triggers stop which
void set_activeSensor(int sensor);   //Used By external functions to set active_sensors
#endif