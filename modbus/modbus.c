/*
 * modbus.c
 *
 *  Created on: 2024Äê3ÔÂ14ÈÕ
 *      Author: Xavier
 */


#include "modbus.h"

void modbus_fsm(modbus_obj_type *obj)
{
    bool new_msg_flag;

    //check new message.
    new_msg_flag = modbus_check_msg(obj->rx_buff + obj->rx_index);
    if(new_msg_flag) obj->rx_index ++;//receive new message.
    if(obj->rx_index >= MSG_BUFF_LENGTH) //overflow.
    {
        obj->error_state = modbus_error_overflow;
        obj->state = modbus_error;
    }

    //main state machine.
    switch(obj->state)
    {
        case modbus_initial_state:
            if(obj->initial_state_flag == false) //first time in initial state
            {
                modbus_start_timer();
                obj->initial_state_flag = true;
            }

            if(new_msg_flag)  modbus_start_timer(); //new message received.

            if(modbus_check_timer_expired()) //timer expired.
            {
                obj->rx_index = 0; //reset index.
                obj->state = modbus_idle;
            }
            break;

        case modbus_idle:
            if(new_msg_flag) //first message received.
            {
                modbus_start_timer();
                obj->state = modbus_reception;
            }
            break;

        case modbus_reception:
            if(new_msg_flag) modbus_start_timer();
            if(modbus_check_timer_expired())
            {
                obj->control_pending_flag = true; //set control pending flag.
                obj->state = modbus_control_and_waiting;
            }
            break;

        case modbus_control_and_waiting:
            if(obj->control_pending_flag == false) //control pending flag has cleared.
            {
                obj->rx_index = 0; //reset index.
                obj->state = modbus_idle;
            }
            break;
        case modbus_error:
            break;
    }
}

//decode messages and operate registers.
void modbus_control(modbus_obj_type *obj)
{

    uint16_t function_code = 0;

    if(obj->control_pending_flag == false) //no control pending.
        return;

    //control pending.
    function_code = obj->rx_buff[0];
    switch(function_code)
    {
        case modbus_read_holding_registers:
            modbus_read_holding_registers_callbackfcn(obj);
            break;
        case modbus_write_single_register:
            break;
        default:
            obj->exception_code = 0x01;
            break;
    }

    obj->control_pending_flag = false;
    return;
}

void modbus_read_holding_registers_callbackfcn(modbus_obj_type *obj)
{
    uint16_t start_ads;
    uint16_t quality;
    uint16_t read_cnt = 0;
    uint16_t tmp_read_value = 0;
    start_ads = ((uint16_t)obj->rx_buff[1]) << 8 + (uint16_t)obj->rx_buff[2];
    quality = ((uint16_t)obj->rx_buff[3]) << 8 + (uint16_t)obj->rx_buff[4];

    if(modbus_check_ads_isvalid(start_ads,quality) == false) //address not valid
    {
        obj->exception_code = 0x02;
        return;
    }

    //address valid
    obj->tx_buff[0] = (char)modbus_read_holding_registers; //function code.
    obj->tx_buff[1] = (char)(quality & 0xFF);

    read_cnt = 0;
    for(read_cnt=0;read_cnt<quality;read_cnt++) //read registers
    {
        tmp_read_value = *(obj->reg[start_ads + read_cnt]);
        obj->tx_buff[2+read_cnt*2] = (char)((tmp_read_value >> 8) & 0xFF);//read register hi
        obj->tx_buff[3+read_cnt*2] = (char)((tmp_read_value) & 0xFF);//read register hi
    }
    obj->tx_index = 2 + quality*2;
}

bool modbus_check_ads_isvalid(uint16_t start_ads,uint16_t quality)
{
    if((uint32_t)start_ads >= (uint32_t)REGISTER_LENGTH) return false; //start address not valid.
    if((uint32_t)quality < (uint32_t)0x1) return false; //quality not valid.
    if((uint32_t)quality > (uint32_t)0x7D) return false; //quality not valid.

    if((uint32_t)(start_ads + quality) >= (uint32_t)REGISTER_LENGTH) return false; //quality not valid.
    return true; //valid
}


