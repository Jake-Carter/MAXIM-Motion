#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "USFSMAX.h"
#include "i2c_helper.h"
#include "utils.h"

//************************************************
// FUNCTION IMPLEMENTATIONS
//************************************************

void USFSMAX_init() {
	printf("Initializing USFSMAX Coprocessor...\n");
	uint8_t status = i2c_read_byte(USFSMAX_ADDR, FUSION_STATUS);
	delay(100);

	if (status == 0) { // No errors/alarms
		USFSMAX_stop_fusion();
		delay(100);

		CoProcessorConfig_t config; // Set up config struct from values set in config.h
		config.cal_points        = CAL_POINTS;
		config.Ascale            = ACC_SCALE;
		config.AODR              = ACC_ODR;
		config.Alpf              = LSM6DSM_ACC_DLPF_CFG;
		config.Ahpf              = LSM6DSM_ACC_DHPF_CFG;
		config.Gscale            = GYRO_SCALE;
		config.GODR              = GYRO_ODR;
		config.Glpf              = LSM6DSM_GYRO_DLPF_CFG;
		config.Ghpf              = LSM6DSM_GYRO_DHPF_CFG;
		config.Mscale            = MAG_SCALE;
		config.MODR              = MAG_ODR;
		config.Mlpf              = LIS2MDL_MAG_LPF;
		config.Mhpf              = LIS2MDL_MAG_HPF;
		config.Pscale            = BARO_SCALE;
		config.PODR              = BARO_ODR;
		config.Plpf              = LPS22HB_BARO_LPF;
		config.Phpf              = LPS22HB_BARO_HPF;
		config.AUX1scale         = AUX1_SCALE;
		config.AUX1ODR           = AUX1_ODR;
		config.AUX1lpf           = AUX1_LPF;
		config.AUX1hpf           = AUX1_HPF;
		config.AUX2scale         = AUX2_SCALE;
		config.AUX2ODR           = AUX2_ODR;
		config.AUX2lpf           = AUX2_LPF;
		config.AUX2hpf           = AUX2_HPF;
		config.AUX3scale         = AUX3_SCALE;
		config.AUX3ODR           = AUX3_ODR;
		config.AUX3lpf           = AUX3_LPF;
		config.AUX3hpf           = AUX3_HPF;
		config.m_v               = M_V;
		config.m_h               = M_H;
		config.m_dec             = MAG_DECLINIATION;
		config.quat_div          = QUAT_DIV;

		// Send configuration to device
		USFSMAX_set_config(config);

		// Restart sensor fusion
		printf("Restarting sensor fusion...");
		USFSMAX_start_fusion();

		// Poll FUSION_STATUS register to see if fusion has resumed, with a timeout of 2s
		printf("Verifying...");

		int count = 0;
		status = i2c_read_byte(USFSMAX_ADDR, FUSION_STATUS);
		while (!(status & FUSION_RUNNING_MASK) && count < 2000) {
			delay(100);
			count += 100;
			status = i2c_read_byte(USFSMAX_ADDR, FUSION_STATUS);
		}

		if (status == 1) { printf("Fusion loop started!\n"); }
		else { printf("Failed to start fusion loop!\n"); return; }

		// Check for sensor errors
		status = i2c_read_byte(USFSMAX_ADDR, SENS_ERR_STAT);
		delay(100);
		if (status != 0) {
			printf("Sensor error!  Status : %i\n", status);
			// TODO : Deal with sensor errors.
			return;
		}

		// Enable DHI corrector
		if(ENABLE_DHI_CORRECTOR)
		  {
		    if(USE_2D_DHI_CORRECTOR)
		    {
		      i2c_write_byte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x50); // Enable DHI corrector, 2D (0x10|0x50)
		      delay(100);
		    } else
		    {
		      i2c_write_byte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x10); // Enable DHI corrector, 3D (0x10)
		      delay(100);
		    }
		  }

		printf("USFSMAX Coprocessor configured!\n\n");

		// Read calibration status/data
		printf("Reading calibration status...\n");
		uint8_t cal_status = i2c_read_byte(USFSMAX_ADDR, CALIBRATION_STATUS);
		if (!(cal_status & 0xF8)) {
			if (!(cal_status & 0x80)) { printf("Invalid Hard Iron offsets!\n"); }
			else if (!(cal_status & 0x40)) { printf("Invalid Fine Magnetometer calibration!\n"); }
			else if (!(cal_status & 0x20)) { printf("Invalid Accelerometer calibration!\n"); }
			else if (!(cal_status & 0x10)) { printf("Invalid Elliptical Magnetometer calibration!\n"); }
			else if (!(cal_status & 0x08)) { printf("Invalid Gyroscope calibration!\n"); }
			printf("The device needs to be calibrated!\n");
		} else {
			printf("Calibration status OK!\n");
		}

		full_adv_cal_t gyro_cal = USFSMAX_get_gyro_cal();
		printf("Gyro calibration : \n");
		printf("Calibration good : %i\n", gyro_cal.cal_good);
		printf("X : %.2f Y : %.2f Z : %.2f\n", gyro_cal.V[0], gyro_cal.V[1], gyro_cal.V[2]);
		delay(100);

		full_adv_cal_t accel_cal = USFSMAX_get_accel_cal();
		printf("Accelerometer calibration : \n");
		printf("Calibration good : %i\n", accel_cal.cal_good);
		printf("X : %.2f Y : %.2f Z : %.2f\n", accel_cal.V[0], accel_cal.V[1], accel_cal.V[2]);
		delay(100);

		full_adv_cal_t magn_cal_ellip = USFSMAX_get_magn_cal_ellip();
		printf("Magnetometer (elliptical) calibration : \n");
		printf("Calibration good : %i\n", magn_cal_ellip.cal_good);
		printf("X : %.2f Y : %.2f Z : %.2f\n", magn_cal_ellip.V[0], magn_cal_ellip.V[1], magn_cal_ellip.V[2]);
		delay(100);

		full_adv_cal_t magn_cal_fine = USFSMAX_get_magn_cal_fine();
		printf("Magnetometer (fine) calibration : \n");
		printf("Calibration good : %i\n", magn_cal_fine.cal_good);
		printf("X : %.2f Y : %.2f Z : %.2f\n", magn_cal_fine.V[0], magn_cal_fine.V[1], magn_cal_fine.V[2]);
		delay(100);

	} else { // Errors/alarms present
		printf("Sensor status error! Code : %i\n", status);
	}
}

void USFSMAX_set_config(CoProcessorConfig_t config) {
	// Unpack configuration struct to a sequence of bytes
	printf("Unpacking config struct...");
	uint8_t* unpacked = malloc(sizeof(CoProcessorConfig_t));
	memcpy(unpacked, &config, sizeof(CoProcessorConfig_t));
	printf("Done!\n");

	// Write configuration block to device
	printf("Writing configuration to device...");
	i2c_write_bytes(USFSMAX_ADDR, COPRO_CFG_DATA0, sizeof(CoProcessorConfig_t), unpacked);
	delay(100);
	printf("Done!\n");

	free(unpacked); // Don't forget to free your memory allocation!!
}

CoProcessorConfig_t USFSMAX_get_config() {
	uint8_t* bytes = malloc(sizeof(CoProcessorConfig_t));
	CoProcessorConfig_t config;

	i2c_read_bytes(USFSMAX_ADDR, COPRO_CFG_DATA0, sizeof(CoProcessorConfig_t), bytes);
	memcpy(&config, bytes, sizeof(CoProcessorConfig_t));

	free(bytes);
	return config;
}

void USFSMAX_start_fusion() {
	i2c_write_byte(USFSMAX_ADDR, FUSION_START_STOP, 0x1 | (OUTPUT_EULER_ANGLES << 1) | (SCALED_SENSOR_DATA << 2));
	delay(100);
}

void USFSMAX_stop_fusion() {
	i2c_write_byte(USFSMAX_ADDR, FUSION_START_STOP, 0x0 | (OUTPUT_EULER_ANGLES << 1) | (SCALED_SENSOR_DATA << 2));
	delay(100);
}

GyroData_t USFSMAX_get_gyro() {
	uint8_t bytes[6];
	GyroData_t gyro_data;

	i2c_read_bytes(USFSMAX_ADDR, G_X_L, 6, bytes);
	gyro_data.x = ((int16_t)bytes[1] << 8) | bytes[0];
	gyro_data.y = ((int16_t)bytes[3] << 8) | bytes[2];
	gyro_data.z = ((int16_t)bytes[5] << 8) | bytes[4];

	return gyro_data;
}

AccelData_t USFSMAX_get_accel() {
	uint8_t bytes[6];
	AccelData_t accel_data;

	i2c_read_bytes(USFSMAX_ADDR, A_X_L, 6, bytes);
	accel_data.x = ((int16_t)bytes[1] << 8) | bytes[0];
	accel_data.y = ((int16_t)bytes[3] << 8) | bytes[2];
	accel_data.z = ((int16_t)bytes[5] << 8) | bytes[4];

	return accel_data;
}

MagnData_t USFSMAX_get_magn() {
	uint8_t bytes[6];
	MagnData_t magn_data;

	i2c_read_bytes(USFSMAX_ADDR, M_X_L, 6, bytes);
	magn_data.x = ((int16_t)bytes[1] << 8) | bytes[0];
	magn_data.y = ((int16_t)bytes[3] << 8) | bytes[2];
	magn_data.z = ((int16_t)bytes[5] << 8) | bytes[4];

	return magn_data;
}

MagnData_Inplane_t USFSMAX_get_magn_inplane() {
	uint8_t bytes[4];
	MagnData_Inplane_t magn_data_inplane;

	i2c_read_bytes(USFSMAX_ADDR, MX_L, 4, MX_L);
	magn_data_inplane.x = (float)(((int16_t)bytes[1] << 8) | bytes[0]) * LIS2MDL_UT_PER_COUNT;
	magn_data_inplane.y = (float)(((int16_t)bytes[3] << 8) | bytes[2]) * LIS2MDL_UT_PER_COUNT;

	return magn_data_inplane;
}

uint32_t USFSMAX_get_baro() {
	uint8_t bytes[3];

	i2c_read_bytes(USFSMAX_ADDR, BARO_XL, 3, bytes);
	return ((int32_t)bytes[2] << 16 | (int32_t)bytes[1] << 8 | bytes[0]);
}

QuatData_t USFSMAX_get_quat() {
	uint8_t bytes[16];
	QuatData_t quat_data;

	i2c_read_bytes(USFSMAX_ADDR, Q0_BYTE0, 16, bytes);
	quat_data.a = bytes_to_float(&bytes[0]);
	quat_data.b = bytes_to_float(&bytes[4]);
	quat_data.c = bytes_to_float(&bytes[8]);
	quat_data.d = bytes_to_float(&bytes[12]);

	return quat_data;
}

EulerData_t USFSMAX_get_euler() {
	uint8_t bytes[16];
	EulerData_t euler_data;

	i2c_read_bytes(USFSMAX_ADDR, YAW_BYTE0, 12, bytes);
	euler_data.heading = bytes_to_float(&bytes[0]);
	euler_data.pitch = bytes_to_float(&bytes[4]);
	euler_data.roll = bytes_to_float(&bytes[8]);

	return euler_data;
}

LinAccelData_t USFSMAX_get_linaccel() {
	uint8_t bytes[6];
	LinAccelData_t linaccel_data;

	i2c_read_bytes(USFSMAX_ADDR, LIN_X_L, 6, bytes);
	linaccel_data.x = ((int16_t)bytes[1] << 8) | bytes[0];
	linaccel_data.y = ((int16_t)bytes[3] << 8) | bytes[2];
	linaccel_data.z = ((int16_t)bytes[5] << 8) | bytes[4];

	return linaccel_data;
}

GravData_t USFSMAX_get_grav() {
	uint8_t bytes[6];
	GravData_t grav_data;

	i2c_read_bytes(USFSMAX_ADDR, GRAV_X_L, 6, bytes);
	grav_data.x = ((int16_t)bytes[1] << 8) | bytes[0];
	grav_data.y = ((int16_t)bytes[3] << 8) | bytes[2];
	grav_data.z = ((int16_t)bytes[5] << 8) | bytes[4];

	return grav_data;
}

full_adv_cal_t USFSMAX_get_gyro_cal() {
	uint8_t* bytes = malloc(sizeof(full_adv_cal_t));
	full_adv_cal_t cal_data;

	i2c_read_bytes(USFSMAX_ADDR, GYRO_CAL_DATA0, sizeof(full_adv_cal_t), bytes);
	memcpy(&cal_data, bytes, sizeof(full_adv_cal_t));

	free(bytes);
	return cal_data;
}

full_adv_cal_t USFSMAX_get_accel_cal() {
	uint8_t* bytes = malloc(sizeof(full_adv_cal_t));
	full_adv_cal_t cal_data;

	i2c_read_bytes(USFSMAX_ADDR, ACCEL_CAL_DATA0, sizeof(full_adv_cal_t), bytes);
	memcpy(&cal_data, bytes, sizeof(full_adv_cal_t));

	free(bytes);
	return cal_data;
}

full_adv_cal_t USFSMAX_get_magn_cal_ellip() {
	uint8_t* bytes = malloc(sizeof(full_adv_cal_t));
	full_adv_cal_t cal_data;

	i2c_read_bytes(USFSMAX_ADDR, ELLIP_MAG_CAL_DATA0, sizeof(full_adv_cal_t), bytes);
	memcpy(&cal_data, bytes, sizeof(full_adv_cal_t));

	free(bytes);
	return cal_data;
}

full_adv_cal_t USFSMAX_get_magn_cal_fine() {
	uint8_t* bytes = malloc(sizeof(full_adv_cal_t));
	full_adv_cal_t cal_data;

	i2c_read_bytes(USFSMAX_ADDR, FINE_MAG_CAL_DATA0, sizeof(full_adv_cal_t), bytes);
	memcpy(&cal_data, bytes, sizeof(full_adv_cal_t));

	free(bytes);
	return cal_data;
}

void USFSMAX_reset_DHI() {
	if (USE_2D_DHI_CORRECTOR) {
		i2c_write_byte(USFSMAX_ADDR, CALIBRATION_REQUEST, 0x70);	// DHI Enable = true, DHI Reset = true, 2D corrector = true
	} else {
		i2c_write_byte(USFSMAX_ADDR, CALIBRATION_REQUEST, 0x30);	// DHI Enable = true, DHI Reset = true, 2D corrector = false
	}
}

float USFSMAX_get_DHI_fit() {
	uint8_t bytes[2];

	i2c_read_bytes(USFSMAX_ADDR, DHI_RSQ_L, 2, bytes);
	return (float)(((int16_t)bytes[1] << 8) | bytes[0]) / 10000.0f;
}