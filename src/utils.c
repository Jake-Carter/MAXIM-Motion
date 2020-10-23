/*
 * utils.c
 *
 *  Created on: Sep 29, 2020
 *      Author: Jake.Carter
 */

#include "utils.h"
#include "mxc_delay.h"

void delay(int ms) {
	MXC_Delay(MXC_DELAY_MSEC(ms));
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
