#include "pneumatic_task.hpp"

extern int current_task;
extern int rotation_ok[2];
bool sensor_array[5] = {0,0,0,0,0};
int active_sensors[5] = {1,2,3,4,5};

void sensor_Init()
{
    for(int i=22;i<27;i++)
        pinMode(i,INPUT);
}
#ifdef ARUDINO_MEGA
void TIM_IRQ_Init()
{
    MsTimer2::set(UPDATE_FRQ,sensors_update);
    MsTimer2::start();
}

void TIM_Stop()
{
    MsTimer2::stop();
}
#endif

#ifdef ARDUINO_DUE
void TIM_IRQ_Init()
{
    Timer1.attachInterrupt(sensors_update);
    Timer1.start(UPDATE_FRQ*1000);
}

void TIM_Stop()
{
    Timer1.stop();
}
#endif


//Update all active sensors; when a sensor is triggered deactivate it
//After the actions are taken sensor can be activated.
void sensor_update()
{
    for(int i=0;i<5;i++)
    {
        if(active_sensors[i]=ACTIVATE)
        {
            sensor_array[i] = !digitalRead(active_sensors[i]+SHIFTER-1);
            if(sensor_array[i]==1) //Sensor is triggered
                active_sensors[i] = DEACTIVATE;
        }
    }
   
}

//set active sensors when external functions need to
void set_activeSensor(int sensor)
{
    active_sensors[sensor-SHIFTER-1] = ACTIVATE+sensor-SHIFTER-1;
}