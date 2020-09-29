/*
 * utils.c
 *
 *  Created on: Sep 29, 2020
 *      Author: Jake.Carter
 */

#include "utils.h"

void delay(int ms) {
	TMR_Delay(MXC_TMR0, MSEC(ms), 0);
}

float bytes_to_float (uint8_t *buf) {
  union
  {
    uint32_t ui32;
    float f;
  } u;

  u.ui32 = (((uint32_t)buf[0]) +
           (((uint32_t)buf[1]) <<  8) +
           (((uint32_t)buf[2]) << 16) +
           (((uint32_t)buf[3]) << 24));
  return u.f;
}
