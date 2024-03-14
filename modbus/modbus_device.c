/*
 * modbus_device.c
 *
 *  Created on: 2024Äê3ÔÂ14ÈÕ
 *      Author: Xavier
 */


#include "modbus_device.h"

void start_modbus_timer(void)
{
    EPWM_setTimeBaseCounterMode(modbus_timer_BASE,EPWM_COUNTER_MODE_STOP_FREEZE); //stop timer
    EPWM_setTimeBaseCounter(modbus_timer_BASE,0);  //reset counter value
    EPWM_clearEventTriggerInterruptFlag(modbus_timer_BASE);  //clear flag.
    EPWM_clearEventTriggerInterruptFlag(modbus_timer_BASE);  //clear flag in case there is a pending flag.
    EPWM_setTimeBaseCounterMode(modbus_timer_BASE,EPWM_COUNTER_MODE_UP);  //start timer
}

bool get_modbus_timer_expired(void)
{
    bool flag;
    flag = EPWM_getEventTriggerInterruptStatus(modbus_timer_BASE); //get timer expired flag.
    if(flag) //if timer expired
    {
        EPWM_setTimeBaseCounterMode(modbus_timer_BASE,EPWM_COUNTER_MODE_STOP_FREEZE); //stop timer
        EPWM_setTimeBaseCounter(modbus_timer_BASE,0);  //reset counter value
        EPWM_clearEventTriggerInterruptFlag(modbus_timer_BASE);  //clear flag.
        EPWM_clearEventTriggerInterruptFlag(modbus_timer_BASE);  //clear flag in case there is a pending flag.
    }
    return flag;

}


