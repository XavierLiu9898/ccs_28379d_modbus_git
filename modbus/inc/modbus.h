/*
 * modbus.h
 *
 *  Created on: 2024��3��14��
 *      Author: Xavier
 */

#ifndef MODBUS_INC_MODBUS_H_
#define MODBUS_INC_MODBUS_H_

#include "modbus_device.h"
#include "crc.h"

typedef enum
{
	modbus_adu,
	modbus_pdu,
} modbus_data_unit_mode_e;

typedef enum {
  modbus_initial_state = 0,
  modbus_idle,
  modbus_emission,
  modbus_control_and_waiting,
  modbus_reception,
  modbus_error,
} modbus_state_e;

typedef enum {
  modbus_error_no_error = 0,
  modbus_error_overflow,
} modbus_error_state_e;

typedef enum {
  modbus_read_holding_registers = 0x03,
  modbus_write_single_register = 0x06,
  modbus_write_multiple_registers = 0x10,
} modbus_function_code_e;

#define MODBUS_DATA_UNIT_MODE modbus_adu
#if MODBUS_DATA_UNIT_MODE == modbus_adu
#define MODBUS_SLAVE_ADDRESS 0x01
#endif

typedef struct {
  modbus_state_e state;
  modbus_error_state_e error_state;

  bool initial_state_flag; // flag set when first time in initial
  bool tx_rdy_flag;
  uint16_t exception_code;

  char rx_buff[MSG_BUFF_LENGTH];
  uint16_t rx_index;

  char tx_buff[MSG_BUFF_LENGTH];
  uint16_t tx_index;

  uint16_t *reg[REGISTER_LENGTH];
} modbus_obj_type;

#define modbus_char_to_uint16(hi, lo)                                          \
  ((((uint16_t)(hi)) << 8) + ((uint16_t)(lo)))

#define modbus_uint16_to_char(val, hi, lo)                                     \
  {                                                                            \
    hi = (char)(((uint16_t)(val) >> 8) & 0xFF);                                \
    lo = (char)(((uint16_t)(val) >> 0) & 0xFF);                                \
  }

extern modbus_obj_type modbus_obj;

extern void modbus_fsm(modbus_obj_type *obj);
extern void modbus_control(modbus_obj_type *obj);

extern void modbus_read_holding_registers_callbackfcn(modbus_obj_type *obj);
extern void modbus_write_single_register_callbackfcn(modbus_obj_type *obj);
extern void modbus_write_multiple_registers_callbackfcn(modbus_obj_type *obj);

extern bool modbus_check_quality_isvalid(uint16_t quality);
extern bool modbus_check_ads_isvalid(uint16_t ads);
#endif /* MODBUS_INC_MODBUS_H_ */
