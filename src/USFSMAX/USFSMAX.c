#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "USFSMAX.h"
#include "i2c_helper.h"
#include "tmr_utils.h"

// TODO : Implement all USFSMAX.h functions

void delay(int ms) {
	TMR_Delay(MXC_TMR0, MSEC(ms), 0);
}

void USFSMAX_init() {
	printf("Initializing USFSMAX Coprocessor...\n");
	uint8_t status = i2c_readByte(USFSMAX_ADDR, FUSION_STATUS);
	delay(100);

	if (status == 0) { // No errors/alarms
		i2c_writeByte(USFSMAX_ADDR, FUSION_START_STOP, 0x0); // Stop the sensor fusion.  Configuration can only be written if fusion loop is stopped.
		delay(100);

		CoProcessorConfig_t config; // Set up config struct from values set in config.h (I don't really like the way this is done but it's how it's done in the original source.  TODO : Do this better
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

		// Unpack configuration struct to a sequence of bytes
		printf("Unpacking config struct...");
		uint8_t* unpacked = malloc(sizeof(CoProcessorConfig_t));
		memcpy(unpacked, &config, sizeof(CoProcessorConfig_t));
		printf("Done!\n");

		// Write configuration block to device
		printf("Writing configuration to device...");
		i2c_writeBytes(USFSMAX_ADDR, COPRO_CFG_DATA0, 30, unpacked);
		delay(100);
		i2c_writeBytes(USFSMAX_ADDR, COPRO_CFG_DATA1, sizeof(CoProcessorConfig_t) - 30, &unpacked[30]);
		delay(100);
		printf("Done!\n");

		free(unpacked); // Don't forget to free your memory allocation!!

		// Restart sensor fusion
		printf("Restarting sensor fusion...");
		i2c_writeByte(USFSMAX_ADDR, FUSION_START_STOP, 0x1);
		delay(100);

		// Poll FUSION_STATUS register to see if fusion has resumed, with a timeout of 2s
		printf("Verifying...");

		int count = 0;
		status = i2c_readByte(USFSMAX_ADDR, FUSION_STATUS);
		while (!(status & FUSION_RUNNING_MASK) && count < 2000) {
			delay(10);
			count += 10;
			status = i2c_readByte(USFSMAX_ADDR, FUSION_STATUS);
		}

		if (status == 0) { printf("Fusion loop started!\n"); }
		else { printf("Failed to start fusion loop!\n"); return; }

		// Check for sensor errors
		status = i2c_readByte(USFSMAX_ADDR, SENS_ERR_STAT);
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
		      i2c_writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x50); // Enable DHI corrector, 2D (0x10|0x50)
		      delay(100);
		    } else
		    {
		      i2c_writeByte(MAX32660_SLV_ADDR, CALIBRATION_REQUEST, 0x10); // Enable DHI corrector, 3D (0x10)
		      delay(100);
		    }
		  }

		printf("USFSMAX Coprocessor configured!\n");
		// TODO : Configuration done, read sensor calibrations and print

	} else { // Errors/alarms present
		printf("Sensor status error! Code : %i\n", status);
	}
}
