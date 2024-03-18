/*
 * modbus.c
 *
 *  Created on: 2024��3��14��
 *      Author: Xavier
 */

#include "modbus.h"

modbus_obj_type modbus_obj;

void modbus_fsm(modbus_obj_type *obj)
{
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
	switch (obj->state)
	{
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
		if (obj->tx_rdy_flag)
			obj->state = modbus_emission;
		if (new_msg_flag) // first message received.
		{
			modbus_start_timer();
			obj->state = modbus_reception;
		}
		break;

	case modbus_reception:
		if (new_msg_flag)
			modbus_start_timer();
		if (modbus_check_timer_expired() == true)
		{
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
		modbus_send_msg(obj->tx_buff, obj->tx_index);
		obj->tx_rdy_flag = false;
		obj->rx_index = 0; // reset index.
		obj->state = modbus_idle;
		break;
	}
}

// decode messages and operate registers.
void modbus_control(modbus_obj_type *obj)
{

#if MODBUS_DATA_UNIT_MODE == modbus_adu
	/*CRC16 check*/
	//CRCchecking
	uint16_t crc_code;
	uint16_t crc_code_rx;
	crc_code = CRC16(obj->rx_buff, obj->rx_index - 2);
	crc_code_rx = modbus_char_to_uint16(obj->rx_buff[obj->rx_index - 2],
										obj->rx_buff[obj->rx_index - 1]);
	if (crc_code != crc_code_rx)
		return;
#endif

	/*get function code*/
	modbus_function_code_e function_code;
#if MODBUS_DATA_UNIT_MODE == modbus_adu
	function_code = (modbus_function_code_e) obj->rx_buff[1];
#else
	function_code = (modbus_function_code_e) obj->rx_buff[0];
#endif

	/*switch function code callback function*/
	switch (function_code)
	{
	case modbus_read_holding_registers:
		modbus_read_holding_registers_callbackfcn(obj);
		break;
	case modbus_write_single_register:
		modbus_write_single_register_callbackfcn(obj);
		break;
	case modbus_write_multiple_registers:
		modbus_write_multiple_registers_callbackfcn(obj);
		break;
	default:
		obj->exception_code = 0x01;
		break;
	}

	if (obj->exception_code != 0x0) // error data received.
	{
		obj->tx_buff[0] = (char) ((uint16_t) (function_code + 0x80) & 0xFF); // error function code.
		obj->tx_buff[1] = (char) (obj->exception_code & 0xFF);
		obj->tx_index = 2;
	}

#if MODBUS_DATA_UNIT_MODE == modbus_adu
	obj->tx_buff[0] = MODBUS_SLAVE_ADDRESS;
	/*CRC16 calculate*/
	crc_code = CRC16(obj->tx_buff, obj->tx_index);
	modbus_uint16_to_char(crc_code, obj->tx_buff[obj->tx_index],
							obj->tx_buff[obj->tx_index + 1]);
	obj->tx_index += 2;
#endif



	obj->tx_rdy_flag = true; // set tx flag to send message.
	return;
}

// this function deals with read holding registers callbacks.

void modbus_read_holding_registers_callbackfcn(modbus_obj_type *obj)
{

	uint16_t start_ads;
	uint16_t quality;
	uint16_t read_cnt = 0;
	uint16_t tmp_read_value = 0;
	char *rx_start;
	char *tx_start;

	/*get message start index*/
#if MODBUS_DATA_UNIT_MODE == modbus_adu
	//skip address byte.
	rx_start = obj->rx_buff + 1;
	tx_start = obj->tx_buff +1;
#else
	//no address byte.
	rx_start = obj->rx_buff;
	tx_start = obj->tx_buff;
#endif

	/*get register start address and quality*/
	start_ads = modbus_char_to_uint16(rx_start[1], rx_start[2]);
	quality = modbus_char_to_uint16(rx_start[3], rx_start[4]);

	/*check register start address and quality*/
	if (modbus_check_quality_isvalid(quality) == false) // quality not valid
	{
		obj->exception_code = 0x03;
		return;
	}

	if (modbus_check_ads_isvalid(start_ads) == false) // start address not valid
	{
		obj->exception_code = 0x02;
		return;
	}

	if (modbus_check_ads_isvalid(start_ads + quality - 1) == false) // end address not valid
	{
		obj->exception_code = 0x02;
		return;
	}

	obj->exception_code = 0; // all data valid.

	/*update transmit buff*/
	tx_start[0] = (char) modbus_read_holding_registers; // function code.
	tx_start[1] = (char) ((quality * 2) & 0xFF);              // quality.
	read_cnt = 0;
	// DINT;
	for (read_cnt = 0; read_cnt < quality; read_cnt++) // read registers
	{
		tmp_read_value = *(obj->reg[start_ads + read_cnt]);
		tx_start[2 + read_cnt * 2] = (char) ((tmp_read_value >> 8) & 0xFF); // read register high.
		tx_start[3 + read_cnt * 2] = (char) ((tmp_read_value) & 0xFF); // read register low
	}
	// EINT;

	obj->tx_index = 2 + quality * 2;
#if MODBUS_DATA_UNIT_MODE == modbus_adu
	obj->tx_index +=1; //address bit.
#endif
}

void modbus_write_single_register_callbackfcn(modbus_obj_type *obj)
{
	uint16_t ads; // write register address.
	uint16_t val; // write register value.

	ads = modbus_char_to_uint16(obj->rx_buff[1], obj->rx_buff[2]);
	val = modbus_char_to_uint16(obj->rx_buff[3], obj->rx_buff[4]);

	if (modbus_check_ads_isvalid(ads) == false) // address not valid
	{
		obj->exception_code = 0x02;
		return;
	}

	obj->exception_code = 0; // all data valid.
	// write register
	DINT;
	*obj->reg[ads] = val;
	EINT;
	// update tx buff.
	obj->tx_buff[0] = (char) modbus_write_single_register; // fcn code.
	obj->tx_buff[1] = (char) ((ads >> 8) & 0xFF);
	obj->tx_buff[2] = (char) ((ads) & 0xFF);
	obj->tx_buff[3] = (char) ((val >> 8) & 0xFF);
	obj->tx_buff[4] = (char) ((val) & 0xFF);
	obj->tx_index = 5;
}

void modbus_write_multiple_registers_callbackfcn(modbus_obj_type *obj)
{
	uint16_t start_ads;
	uint16_t quality;
	uint16_t byte_cnt;
	uint16_t write_cnt;
	uint16_t tmp_write_value;

	start_ads = modbus_char_to_uint16(obj->rx_buff[1], obj->rx_buff[2]); // start address.
	quality = modbus_char_to_uint16(obj->rx_buff[3], obj->rx_buff[4]); // quality.
	byte_cnt = (uint16_t) (obj->rx_buff[5]); // byte count.

	if ((modbus_check_quality_isvalid(quality) == false) || // is quality valid?
			(quality * 2 != byte_cnt))
	{ // is byte count valid?
		obj->exception_code = 0x03;
		return;
	}

	if ((modbus_check_ads_isvalid(start_ads) == false) || // start address valid?
			modbus_check_ads_isvalid(start_ads + quality - 1) ==
			false)
	{ // end address not valid
		obj->exception_code = 0x02;
		return;
	}
	// write multiple registers.
	DINT;
	for (write_cnt = 0; write_cnt < quality; write_cnt++)
	{
		tmp_write_value = modbus_char_to_uint16(
				obj->rx_buff[6 + 2 * write_cnt],
				obj->rx_buff[7 + 2 * write_cnt]);
		*obj->reg[start_ads + write_cnt] = tmp_write_value;
	}
	EINT;

	// update tx buff
	obj->tx_buff[0] = (char) modbus_write_multiple_registers; // fcn code.
	modbus_uint16_to_char(start_ads, obj->tx_buff[1], obj->tx_buff[2]); // starting address.
	modbus_uint16_to_char(quality, obj->tx_buff[3], obj->tx_buff[4]); //  quality.
	obj->tx_index = 5;
}

bool modbus_check_quality_isvalid(uint16_t quality)
{
	if ((quality < 0x1) || (quality > 0x7D))
		return false; // quality not valid.
	return true;
}
bool modbus_check_ads_isvalid(uint16_t ads)
{
	if (ads >= REGISTER_LENGTH)
		return false; // address not valid.
	return true;    // valid
}
