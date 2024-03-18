#ifndef MODBUS_INC_MODBUS_USER_REG_H_
#define MODBUS_INC_MODBUS_USER_REG_H_

#define MODBUS_RGS_OFFSET_USER_REG 0x0100
typedef enum {
  modbus_reg_index_user_reg_0 = MODBUS_RGS_OFFSET_USER_REG + 0x0001,
  modbus_reg_index_user_reg_1 = MODBUS_RGS_OFFSET_USER_REG + 0x0002,
  modbus_reg_index_user_reg_2 = MODBUS_RGS_OFFSET_USER_REG + 0x0003,
  modbus_reg_index_user_reg_3 = MODBUS_RGS_OFFSET_USER_REG + 0x0004,
} modbus_reg_index_user_reg_e;

#endif /*MODBUS_INC_MODBUS_USER_REG_H_*/

