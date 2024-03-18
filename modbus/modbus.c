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
	/*address check*/
	char rx_ads = modbus_read8_rx_buff(obj);
	if (rx_ads != (char) MODBUS_SLAVE_ADDRESS)
		return; //address not match.

	/*CRC16 check*/
	uint16_t crc_code;
	uint16_t crc_code_rx;
	crc_code = CRC16(obj->rx_buff, obj->rx_index - 2); //calculate CRC value.
	crc_code_rx = modbus_char_to_uint16(obj->rx_buff[obj->rx_index - 2],
										obj->rx_buff[obj->rx_index - 1]); //received CRC value.
	if (crc_code != crc_code_rx)
		return; //CRC value not match.

#endif

	/*reset transition index*/
	obj->tx_index = 0;

#if MODBUS_DATA_UNIT_MODE == modbus_adu
	/*load address to tx buff*/
	modbus_load8_tx_buff(obj, rx_ads);
#endif

	/*get function code*/
	modbus_function_code_e function_code;
	function_code = (modbus_function_code_e) modbus_read8_rx_buff(obj);

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

	/*check exception code*/
	if (obj->exception_code != 0x0) // error data received.
	{
		modbus_load8_tx_buff(
				obj, (char) (((uint16_t) function_code + 0x80) & 0xFF));
		modbus_load8_tx_buff(obj, (char) (obj->exception_code & 0xFF));
	}

#if MODBUS_DATA_UNIT_MODE == modbus_adu
	/*CRC16 calculate*/
	crc_code = CRC16(obj->tx_buff, obj->tx_index);
	modbus_load16_tx_buff(obj,crc_code);
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

	/*get register start address and quality*/
	start_ads = modbus_read16_rx_buff(obj);
	quality = modbus_read16_rx_buff(obj);

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

	/*load transmit buff*/
	modbus_load8_tx_buff(obj, (char) modbus_read_holding_registers); //function code
	modbus_load8_tx_buff(obj, (char) ((quality * 2) & 0xFF));        // bytes.

	read_cnt = 0;
	// DINT;
	for (read_cnt = 0; read_cnt < quality; read_cnt++) // read registers
	{
		tmp_read_value = *(obj->reg[start_ads + read_cnt]);
		modbus_load16_tx_buff(obj, tmp_read_value);
	}
	// EINT;
}

void modbus_write_single_register_callbackfcn(modbus_obj_type *obj)
{
	uint16_t ads; // write register address.
	uint16_t val; // write register value.

	ads = modbus_read16_rx_buff(obj);
	val = modbus_read16_rx_buff(obj);

	/*check register start address*/
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

	/* load tx buff.*/
	modbus_load8_tx_buff(obj, (char) modbus_write_single_register); //function code.
	modbus_load16_tx_buff(obj, ads); //address
	modbus_load16_tx_buff(obj, val); //address
}

void modbus_write_multiple_registers_callbackfcn(modbus_obj_type *obj)
{
	uint16_t start_ads;
	uint16_t quality;
	uint16_t byte_cnt;
	uint16_t write_cnt;
	uint16_t tmp_write_value;

	start_ads = modbus_read16_rx_buff(obj); // start address.
	quality = modbus_read16_rx_buff(obj); // quality.
	byte_cnt = (uint16_t) modbus_read8_rx_buff(obj); // byte count.

	/*check register start address and quality*/
	if ((modbus_check_quality_isvalid(quality) == false) || // is quality valid?
			(quality * 2 != byte_cnt))  // is byte count valid?
	{
		obj->exception_code = 0x03;
		return;
	}

	if ((modbus_check_ads_isvalid(start_ads) == false) || // start address valid?
			modbus_check_ads_isvalid(start_ads + quality - 1) == false) // end address valid?
	{
		obj->exception_code = 0x02;
		return;
	}

	// write multiple registers.
	DINT;
	for (write_cnt = 0; write_cnt < quality; write_cnt++)
	{
		tmp_write_value = modbus_read16_rx_buff(obj);
		*obj->reg[start_ads + write_cnt] = tmp_write_value;
	}
	EINT;

	// update tx buff
	modbus_load8_tx_buff(obj, (char) modbus_read_holding_registers); //function code
	modbus_load16_tx_buff(obj,start_ads);
	modbus_load16_tx_buff(obj,quality);
}

void modbus_load8_tx_buff(modbus_obj_type *obj, char val)
{
	obj->tx_buff[obj->tx_index] = val;
	obj->tx_index++;
}

void modbus_load16_tx_buff(modbus_obj_type *obj, uint16_t val)
{
	modbus_load8_tx_buff(obj, (char) ((val >> 8) & 0xFF)); // read register high.
	modbus_load8_tx_buff(obj, (char) ((val) & 0xFF)); // read register low
}

char modbus_read8_rx_buff(modbus_obj_type *obj)
{
	char read_val;
	read_val = obj->rx_buff[obj->rx_head];
	obj->rx_head++;
	return read_val;
}

uint16_t modbus_read16_rx_buff(modbus_obj_type *obj)
{
	char read_val_hi;
	char read_val_lo;
	uint16_t read_val;
	read_val_hi = modbus_read8_rx_buff(obj);
	read_val_lo = modbus_read8_rx_buff(obj);

	read_val = modbus_char_to_uint16(read_val_hi, read_val_lo);
	return read_val;
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

