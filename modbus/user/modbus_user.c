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

  // initial register address.
  obj->reg[0] = (uint16_t *)&(user_modbus_reg[0]) + 1;
  obj->reg[1] = (uint16_t *)&(user_modbus_reg[0]);

}
