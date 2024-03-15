/*
 * modbus_user.c
 *
 *  Created on: 2024��3��14��
 *      Author: Xavier
 */

#include "modbus_user.h"

uint16_t user_modbus_reg[10];

void modbus_user_initial(modbus_obj_type *obj) {
  uint16_t index;

	//initial register address.
  for (index = 0; index < 10; index++) {
    obj->reg[index] = &(user_modbus_reg[index]);
  }
}
