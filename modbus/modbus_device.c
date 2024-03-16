/*
 * modbus_device.c
 *
 *  Created on: 2024��3��14��
 *      Author: Xavier
 */

#include "modbus_device.h"

bool modbus_check_msg(char *msg) {
  uint32_t base = modbus_sci_BASE;

  if (SCI_getRxFIFOStatus(base) == SCI_FIFO_RX0) // no new message.
    return false;
  // new message received.
  *msg = SCI_readCharNonBlocking(base);
  return true;
}

void modbus_start_timer(void) {
  uint16_t base = modbus_timer_BASE;

  EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_STOP_FREEZE); // stop
                                                                    // timer
  EPWM_setTimeBaseCounter(base, 0);          // reset counter value
  EPWM_clearEventTriggerInterruptFlag(base); // clear flag.
  EPWM_clearEventTriggerInterruptFlag(
      base); // clear flag in case there is a pending flag.
  EPWM_setTimeBaseCounterMode(base, EPWM_COUNTER_MODE_UP); // start timer
}

bool modbus_check_timer_expired(void) {
  uint16_t base = modbus_timer_BASE;
  bool flag;
  flag = EPWM_getEventTriggerInterruptStatus(base); // get timer expired flag.
  if (flag)                                         // if timer expired
  {
    EPWM_setTimeBaseCounterMode(base,
                                EPWM_COUNTER_MODE_STOP_FREEZE); // stop timer
    EPWM_setTimeBaseCounter(base, 0);          // reset counter value
    EPWM_clearEventTriggerInterruptFlag(base); // clear flag.
    EPWM_clearEventTriggerInterruptFlag(
        base); // clear flag in case there is a pending flag.
  }
  return flag;
}
