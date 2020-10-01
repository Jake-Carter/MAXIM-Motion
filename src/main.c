/**
 * @file        main.c
 * @brief       I2C Loopback Example 
 * @details     This example uses the I2C Master to read/write from/to the I2C Slave. For
 *              this example you must connect P0.6 to P0.14 (SDA) and P0.7 to P0.15 (SCL). The Master
 *              will use P0.6 and P0.7. The Slave will use P0.14 and P0.15. You must also
 *              connect the pull-up jumpers (JP23 and JP24) to the proper I/O voltage.
 *              Refer to JP27 to determine the I/O voltage.
 * @note        Other devices on the EvKit will be using the same bus. This example cannot be combined with
 *              a PMIC or bluetooth example because the I2C Slave uses GPIO pins for those devices.
 */

/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *
 * $Date: 2019-02-26 15:50:33 -0600 (Tue, 26 Feb 2019) $
 * $Revision: 41252 $
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "mxc_config.h"
#include "pb.h"
#include "i2c_helper.h"
#include "USFSMAX.h"
#include "utils.h"

void init() {
	i2c_init();
	USFSMAX_init();
}

// *****************************************************************************
int main(void)
{
	init();
	int count = 0;

	USFSMAXData_t data;

	do {
		data.gyro = USFSMAX_get_gyro();
		data.accel = USFSMAX_get_accel();
		data.magn = USFSMAX_get_magn();
		data.barom = USFSMAX_get_baro();
		data.quat = USFSMAX_get_quat();
		data.linaccel = USFSMAX_get_linaccel();
		data.grav = USFSMAX_get_grav();

		printf("\nGyroscope data:\nX : %i\tY : %i\t Z : %i\n", data.gyro.x, data.gyro.y, data.gyro.z);
		printf("Accelerometer data:\nX : %i\tY : %i\t Z : %i\n", data.accel.x, data.accel.y, data.accel.z);
		printf("Magnetometer data:\nX : %i\tY : %i\t Z : %i\n", data.magn.x, data.magn.y, data.magn.z);
		printf("Barometer data\nValue : %i\n", data.barom);
		printf("Orientation data (Quaternion):\n %.2f + %.2f*i + %.2f*j + %.2f*k\n", data.quat.a, data.quat.b, data.quat.c, data.quat.d);
		printf("Linear acceleration data:\nX : %i\tY : %i\t Z : %i\n", data.linaccel.x, data.linaccel.y, data.linaccel.z);
		printf("Gravity data:\nX : %i\tY : %i\t Z : %i\n", data.grav.x, data.grav.y, data.grav.z);

		delay(5000);
		count++;
	} while (count < 20);

	USFSMAX_stop_fusion();

    return 1;
}
