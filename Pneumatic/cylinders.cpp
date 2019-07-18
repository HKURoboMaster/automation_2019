#include "pneumatic_task.hpp"

typedef void(*callback)(int,int[]);

Cylinder::Cylinder(int Pin[2],callback drv_func)
{
    pin[0] = Pin[0];
    pin[1] = Pin[1];
    Driver = drv_func;
    pinMode(pin[0],OUTPUT);
    pinMode(pin[1],OUTPUT);
}


// We define the positive direction Here for 
// Direction debugging
void Cylinder::set_Direct(int dir)
{
    direction = dir;
}

// Return Status: On / Off / In Progress
int Cylinder::get_status()
{
    return status;
}

//Move: Launch the actuator accordingly
//Contains an Pointer to driver function:
//Driver function need pin  on / off command
void Cylinder::move(int dir)
{
    if(dir==direction)
    {
        (*Driver)(1,pin);
        status = 1;
    }
    else if(dir==2)
    {
        (*Driver)(2,pin);
        status = 2;
    }
    else 
    {
        (*Driver)(0,pin);
        status = 0;
    }
}

// Below are drivers for different cylinder types
//It supposed to be sent as function pointers

// This is driver for rodless cylinder, which has three state
void rodless_drv(int dir,int pin[])
{
    switch(dir)
    {
        case 0:
            digitalWrite(pin[1],LOW);
            digitalWrite(pin[0],HIGH);
            break;
        case 1:
            digitalWrite(pin[0],LOW);
            digitalWrite(pin[1],HIGH);
            break;
        case 2:
            digitalWrite(pin[0],LOW);
            digitalWrite(pin[1],LOW);
            break;
    }
}

// Normal Driver applicable to any cylinder which has 
// One control pin
void normal_drv(int dir, int pin[])
{
    switch(dir)
    {
        case 0:
            digitalWrite(pin[0],LOW);
            break;
        case 1:
            digitalWrite(pin[0],HIGH);
            break;
    }
}