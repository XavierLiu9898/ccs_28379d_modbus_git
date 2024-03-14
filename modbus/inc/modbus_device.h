/*
 * modbus_device.h
 *
 *  Created on: 2024Äê3ÔÂ14ÈÕ
 *      Author: Xavier
 */

#ifndef MODBUS_INC_MODBUS_DEVICE_H_
#define MODBUS_INC_MODBUS_DEVICE_H_


#include "device.h"
#include "board.h"


extern bool modbus_check_msg(char *msg);
extern void start_modbus_timer(void);
extern bool get_modbus_timer_expired(void);



#endif /* MODBUS_INC_MODBUS_DEVICE_H_ */
