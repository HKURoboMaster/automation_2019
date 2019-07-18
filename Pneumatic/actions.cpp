#include "pneumatic_task.hpp"

//Cylinders are externed in header file.
//Each cylinder was declared in pneumatic.task
extern int active_sensors[5];
extern bool  sensor_array[5];
//extern Cylinder rodless,eject_left,eject_right,grabber,arm;
extern int rotation_flag[2];    //Get from Stm32 indicate the grabber rotation has accomplished.
extern int rotation_ok[2];      //Indicate the STM32 can rotate the grabber
extern int command_flag;
int box_counter = 0;

int rodless_pin[2]      = {RODLESS_LEFT,RODLESS_RIGHT};
int eject_right_pin[2]  = {EJECT_RIGHT,NULL_PIN};
int eject_left_pin[2]   = {EJECT_LEFT,NULL_PIN};
int grabber_pin[2]      = {GRABBER,NULL_PIN};
int arm_pin[2]          = {ARM,NULL_PIN};

//rodless.set_Direct(ROD_SET);
Cylinder rodless(rodless_pin,rodless_drv);
Cylinder eject_left(eject_left_pin,normal_drv);
Cylinder eject_right(eject_right_pin,normal_drv);
Cylinder grabber(grabber_pin,normal_drv);
Cylinder arm(arm_pin,normal_drv);

int counter = 0;  //Prevent the reset action from time over flow

void action_init()
{
    rodless.set_Direct(ROD_SET);
    eject_left.set_Direct(EJECT_SET);
    eject_right.set_Direct(EJECT_SET);
    grabber.set_Direct(GRABBER_GRAB);
    arm.set_Direct(ARM_REACH);
}

bool eject()
{
    if(grabber.get_status()==GRABBER_RELEASE)
    {
    eject_left.move(EJECT_SET); //Release the cylinder to eject
    eject_right.move(EJECT_SET);
    delay(500); // To be adjusted
    eject_left.move(EJECT_RESET);
    eject_right.move(EJECT_RESET);
    }
    else
    {
        delay(500);
    }
    if(eject_left.get_status()==1 && eject_right.get_status()==1)
        return true;
    else
    {
        return false;
    }
}

bool reset()
{
    grabber.move(GRABBER_RELEASE); // First Release the grabber make sure the ejector can be back in position
    eject_left.move(EJECT_RESET);
    eject_right.move(EJECT_RESET);
    arm.move(ARM_RETRACT);
    while(sensor_array[0]==0 && counter<=500)
    {
        rodless.move(ROD_RESET);
    }
    counter = 0; // Reset the counter for next reset action
    rodless.move(ROD_STOP);
    set_activeSensor(SENSOR1);
}


//TASK1 is Grab First and Second Row of Ammo Boxes under the resource Island
//TASK2 is Grab A single box of Ammo on the resource Island
//TASK3 is Grab Continous 3 Boxes of Ammo Boxes on the resource Island
bool grab_task(int flag)
{
     if(flag == TASK2 || flag == TASK3)
    {
        arm.move(ARM_REACH);
        delay(500);
    }
    while(1)
    {
    if(command_flag==RESET_TASK)
    {
        reset();
        break;
    }
    if(command_flag ==EJECT_TASK)
    {
        eject();
    }
    rotation_ok[0] = 1; //Allow stm32 to rotate the claw
    while(!rotation_flag[0]); //Wait until stm32 finish rotation the claw
    grabber.move(GRABBER_GRAB);
    delay(500); //Wait until grab is finished
    rotation_ok[1] = 1;
    while(!rotation_flag[1]);
    grabber.move(GRABBER_RELEASE);
    delay(400);
    eject();
    delay(500);
    box_counter++;
    switch(flag)
    {
        case TASK1: //Only wait for reach specific target
            while(sensor_array[box_counter])
            {
                if(box_counter<=2)
                    rodless.move(ROD_SET);
                else
                {
                    rodless.move(ROD_RESET);
                    delay(500);
                    if(box_counter==3)
                        arm.move(ARM_REACH);
                }
            }
            rodless.move(ROD_STOP);
            break;
        case TASK2:
            break;
        case TASK3:
            while(sensor_array[box_counter])
            {
                rodless.move(ROD_SET);
            }
            rodless.move(ROD_STOP);
            break;
    }
    if((flag==TASK1 && box_counter>=5)||(flag==TASK2)||(flag==TASK3 && box_counter>=3))
    {
        arm.move(ARM_RETRACT);
        break;
    }
    }
    box_counter = 0;  // Reset the number of box counter
    return reset();
}
