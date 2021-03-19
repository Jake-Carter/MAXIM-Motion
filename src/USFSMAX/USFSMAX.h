/*
 * Copyright (c) 2019 Gregory Tomasch.  All rights reserved.

 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#ifndef USFSMAX_h
#define USFSMAX_h

//#include "Alarms.h" // TODO : Implement Alarm functions
#include "def.h"
#include "config.h"
#include "Types.h"
#include "USFSMAX_regdefs.h"

//////////////////////////////////
// STRUCTS
//////////////////////////////////

// Gyroscope data struct
typedef struct {
	int16_t x, y, z;
} GyroData_t;

// Accelerometer data struct
typedef struct {
	int16_t x, y, z;
} AccelData_t;

// Magnetometer data
typedef struct {
	int16_t x, y, z;
} MagnData_t;

typedef struct {
	float x, y;
} MagnData_Inplane_t;

// Orientation data in quaternions 
typedef struct {
	float a, b, c, d;	// a + b*i + c*j + d*k, where i, j, and k are the unit vectors.
} QuatData_t;

// Barometer data
typedef struct {
	float heading, pitch, roll;
} EulerData_t;

// Orientation data in euler angles 
typedef struct {
	int16_t x, y, z;
} LinAccelData_t;

// Linear acceleration data
typedef struct {
	int16_t x, y, z;
} GravData_t;

// Gravity data
typedef struct {
	GyroData_t gyro;
	AccelData_t accel;
	MagnData_t magn;
	uint32_t barom;
	QuatData_t quat; // (if enabled)
	EulerData_t euler; // (if enabled)
	LinAccelData_t linaccel;
	GravData_t grav;
} USFSMAXData_t;

//////////////////////////////////
// FUNCTIONS
//////////////////////////////////

// Management functions
void USFSMAX_init();
void USFSMAX_set_config(CoProcessorConfig_t config);
CoProcessorConfig_t USFSMAX_get_config();
void USFSMAX_start_fusion();
void USFSMAX_stop_fusion();

// Data functions
GyroData_t USFSMAX_get_gyro();
AccelData_t USFSMAX_get_accel();
MagnData_t USFSMAX_get_magn();
MagnData_Inplane_t USFSmax_get_magn_inplane();
uint32_t USFSMAX_get_baro();
QuatData_t USFSMAX_get_quat();
EulerData_t USFSMAX_get_euler();
LinAccelData_t USFSMAX_get_linaccel();
GravData_t USFSMAX_get_grav();

// Calibration functions
full_adv_cal_t USFSMAX_get_gyro_cal();
full_adv_cal_t USFSMAX_get_accel_cal();
full_adv_cal_t USFSMAX_get_magn_cal_ellip();
full_adv_cal_t USFSMAX_get_magn_cal_fine();
void USFSMAX_reset_DHI();
float USFSMAX_get_DHI_fit();

#endif // USFSMAX_h
