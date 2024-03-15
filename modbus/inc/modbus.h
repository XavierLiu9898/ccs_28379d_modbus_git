/*
 * modbus.h
 *
 *  Created on: 2024Äê3ÔÂ14ÈÕ
 *      Author: Xavier
 */

#ifndef MODBUS_INC_MODBUS_H_
#define MODBUS_INC_MODBUS_H_


#include "modbus_device.h"


typedef enum
{
    modbus_initial_state = 0,
    modbus_idle,
    modbus_emission,
    modbus_control_and_waiting,
    modbus_reception,
    modbus_error,
} modbus_state_e;


typedef enum
{
    modbus_error_no_error = 0,
    modbus_error_overflow,
} modbus_error_state_e;

typedef enum
{
    modbus_read_holding_registers,
    modbus_write_single_register,
} modbus_function_code_e;

typedef struct
{
    modbus_state_e state;
    modbus_error_state_e error_state;

    bool initial_state_flag; //flag set when first time in initial
    bool control_pending_flag;
    uint16_t exception_code;

    char rx_buff[MSG_BUFF_LENGTH];
    uint16_t rx_index;

    char tx_buff[MSG_BUFF_LENGTH];
    uint16_t tx_index;

    uint16_t *reg[REGISTER_LENGTH];
} modbus_obj_type;

extern void modbus_fsm(modbus_obj_type *obj);
extern void modbus_control(modbus_obj_type *obj);
extern void modbus_read_holding_registers_callbackfcn(modbus_obj_type *obj);
extern bool modbus_check_ads_isvalid(uint16_t start_ads,uint16_t quality);
#endif /* MODBUS_INC_MODBUS_H_ */
