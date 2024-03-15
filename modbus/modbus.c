/*
 * modbus.c
 *
 *  Created on: 2024��3��14��
 *      Author: Xavier
 */

#include "modbus.h"



modbus_obj_type modbus_obj;

void modbus_fsm(modbus_obj_type *obj) {
  bool new_msg_flag;
  // check new message.
  new_msg_flag = modbus_check_msg(obj->rx_buff + obj->rx_index);
  if (new_msg_flag)
    obj->rx_index++;                    // receive new message.
  if (obj->rx_index >= MSG_BUFF_LENGTH) // overflow.
  {
    obj->error_state = modbus_error_overflow;
    obj->state = modbus_error;
  }

  // main state machine.
  switch (obj->state) {
  case modbus_initial_state:
    if (obj->initial_state_flag == false) // first time in initial state
    {
      modbus_start_timer();
      obj->initial_state_flag = true;
    }

    if (new_msg_flag)
      modbus_start_timer(); // new message received.

    if (modbus_check_timer_expired()) // timer expired.
    {
      obj->rx_index = 0; // reset index.
      obj->state = modbus_idle;
    }
    break;

  case modbus_idle:
    if (new_msg_flag) // first message received.
    {
      modbus_start_timer();
      obj->state = modbus_reception;
    }
    break;

  case modbus_reception:
    if (new_msg_flag)
      modbus_start_timer();
    if (modbus_check_timer_expired() == true) {
      obj->state = modbus_control_and_waiting;
    }
    break;

  case modbus_control_and_waiting:
    modbus_control(obj);
    obj->rx_index = 0; // reset index.
    obj->state = modbus_idle;
    break;
		
  case modbus_error:
    obj->state = modbus_error;
    break;
  case modbus_emission:
		obj->tx_rdy_flag = false;
    obj->rx_index = 0; // reset index.
    obj->state = modbus_idle;		
		break;
  }
}

// decode messages and operate registers.
void modbus_control(modbus_obj_type *obj) {

  modbus_function_code_e function_code;
  function_code = (modbus_function_code_e)obj->rx_buff[0];

  switch (function_code) {
  case modbus_read_holding_registers:
    modbus_read_holding_registers_callbackfcn(obj);
    break;
  case modbus_write_single_register:
    modbus_write_single_register_callbackfcn(obj);
    break;
  default:
    obj->exception_code = 0x01;
    break;
  }

	if(obj->exception_code != 0x0) // error data received.
	{
		obj->tx_buff[0] = (char)((uint16_t)(function_code+0x80) & 0xFF); //error function code.
		obj->tx_buff[1] = (char)(obj->exception_code & 0xFF);
		obj->tx_index = 2;
	}
	obj->tx_rdy_flag = true; //set tx flag to send message.
  return;
}

// this function deals with read holding registers callbacks.

void modbus_read_holding_registers_callbackfcn(modbus_obj_type *obj) {
  uint16_t start_ads;
  uint16_t quality;
  uint16_t read_cnt = 0;
  uint16_t tmp_read_value = 0;
  start_ads = ((uint16_t)obj->rx_buff[1]) << 8 + (uint16_t)obj->rx_buff[2];
  quality = ((uint16_t)obj->rx_buff[3]) << 8 + (uint16_t)obj->rx_buff[4];

  if (modbus_check_quality_isvalid(quality) == false) //quality not valid
  {
    obj->exception_code = 0x03;
    return;
  }

  if (modbus_check_ads_isvalid(start_ads) == false) // start address not valid
  {
    obj->exception_code = 0x02;
    return;
  }

    if (modbus_check_ads_isvalid(start_ads + quality -1) == false) //end address not valid
  {
    obj->exception_code = 0x02;
    return;
  }

	obj->exception_code = 0; //all data valid.

  //update tx buff.
  obj->tx_buff[0] = (char)modbus_read_holding_registers; // function code.
  obj->tx_buff[1] = (char)(quality & 0xFF); //quality.
  read_cnt = 0;
	DINT;
  for (read_cnt = 0; read_cnt < quality; read_cnt++) // read registers
  {
    tmp_read_value = *(obj->reg[start_ads + read_cnt]);
    obj->tx_buff[2 + read_cnt * 2] =
        (char)((tmp_read_value >> 8) & 0xFF); // read register high.
    obj->tx_buff[3 + read_cnt * 2] =
        (char)((tmp_read_value)&0xFF); // read register low
  }
	EINT;
  obj->tx_index = 2 + quality * 2;
}

void modbus_write_single_register_callbackfcn(modbus_obj_type *obj) {
  uint16_t ads; // write register address.
  uint16_t val; // write register value.

  ads = ((uint16_t)(obj->rx_buff[1])) << 8 + (uint16_t)(obj->rx_buff[2]);
  val = ((uint16_t)(obj->rx_buff[3])) << 8 + (uint16_t)(obj->rx_buff[4]);

  if (modbus_check_ads_isvalid(ads) == false) // address not valid
  {
    obj->exception_code = 0x02;
    return;
  }

	obj->exception_code = 0; //all data valid.
  // write register
	DINT;
	*obj->reg[ads] = val;
	EINT;
	// update tx buff.
	obj->tx_buff[0] = (char)modbus_write_single_register; //fcn code.
	obj->tx_buff[1] = (char)((ads >> 8) & 0xFF);
	obj->tx_buff[2] = (char)((ads) & 0xFF);
	obj->tx_buff[3] = (char)((val >> 8) & 0xFF);
	obj->tx_buff[4] = (char)((val) & 0xFF);
	obj->tx_index = 5;
}

bool modbus_check_quality_isvalid(uint16_t quality) {
  if ((quality < 0x1) || (quality > 0x7D))
    return false; // quality not valid.
  return true;
}
bool modbus_check_ads_isvalid(uint16_t ads) {
  if (ads >= REGISTER_LENGTH)
    return false; //address not valid.
  return true;    // valid
}
