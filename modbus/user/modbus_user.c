/*
 * modbus_user.c
 *
 *  Created on: 2024��3��14��
 *      Author: Xavier
 */

#include "modbus_user.h"

float32_t user_modbus_reg[3] = {0.1f, 0.2f, 0.3f};

// this function needs to be called at system starup to initialize the modbus
// register addresses.
void modbus_user_initial(modbus_obj_type *obj) {

  // initial register address. data in MCU is in little-endian.
  obj->reg[0] = (uint16_t *)&(user_modbus_reg[0]); //low two bytes.
  obj->reg[1] = (uint16_t *)&(user_modbus_reg[0]) + 1; //high two bytes
  obj->reg[2] = (uint16_t *)&(user_modbus_reg[1]); //low two bytes.
  obj->reg[3] = (uint16_t *)&(user_modbus_reg[1]) + 1; //high two bytes
  obj->reg[4] = (uint16_t *)&(user_modbus_reg[2]); //low two bytes.
  obj->reg[5] = (uint16_t *)&(user_modbus_reg[2]) + 1; //high two bytes

}
