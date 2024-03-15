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

#define MSG_BUFF_LENGTH 256
#define REGISTER_LENGTH 256

#define mod_bus_send_msg(msg,length) SCI_writeCharArray(modbus_sci_BASE,(uint16_t *)(msg),(uint16_t)(length))


extern bool modbus_check_msg(char *msg);
extern void modbus_start_timer(void);
extern bool modbus_check_timer_expired(void);



#endif /* MODBUS_INC_MODBUS_DEVICE_H_ */
