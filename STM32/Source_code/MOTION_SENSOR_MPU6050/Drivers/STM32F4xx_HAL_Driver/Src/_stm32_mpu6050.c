/**	
 * |----------------------------------------------------------------------
 * | Copyright (c) 2016 Tilen Majerle
 * |  
 * | Permission is hereby granted, free of charge, to any person
 * | obtaining a copy of this software and associated documentation
 * | files (the "Software"), to deal in the Software without restriction,
 * | including without limitation the rights to use, copy, modify, merge,
 * | publish, distribute, sublicense, and/or sell copies of the Software, 
 * | and to permit persons to whom the Software is furnished to do so, 
 * | subject to the following conditions:
 * | 
 * | The above copyright notice and this permission notice shall be
 * | included in all copies or substantial portions of the Software.
 * | 
 * | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * | OTHER DEALINGS IN THE SOFTWARE.
 * |----------------------------------------------------------------------
 */
#include "_stm32_mpu6050.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define MPU6050_I_AM				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C

/*These are the addresses of Mpu6050 from which we will fetch accelerometer x,y,z high and low values */

#define MPU6050_ACCEL_X_HIGH		0x3B
#define MPU6050_ACCEL_X_LOW		0x3C
#define MPU6050_ACCEL_Y_HIGH		0x3D
#define MPU6050_ACCEL_Y_LOW		0x3E
#define MPU6050_ACCEL_Z_HIGH		0x3F
#define MPU6050_ACCEL_Z_LOW		    0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42

/*These are the addresses of Mpu6050 from which we will fetch gyroscope x,y,z high and low values */
#define MPU6050_GYRO_X_HIGH			0x43
#define MPU6050_GYRO_X_LOW			0x44
#define MPU6050_GYRO_Y_HIGH			0x45
#define MPU6050_GYRO_Y_LOW			0x46
#define MPU6050_GYRO_Z_HIGH			0x47
#define MPU6050_GYRO_Z_LOW			0x48
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A


#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENSITIVITY_250		((float) 131)
#define MPU6050_GYRO_SENSITIVITY_500		((float) 65.5)
#define MPU6050_GYRO_SENSITIVITY_1000		((float) 32.8)
#define MPU6050_GYRO_SENSITIVITY_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENSITIVITY_2			((float) 16384)
#define MPU6050_ACCE_SENSITIVITY_4			((float) 8192)
#define MPU6050_ACCE_SENSITIVITY_8			((float) 4096)
#define MPU6050_ACCE_SENSITIVITY_16		    ((float) 2048)


_MPU6050_Result_t _MPU6050_Init(_MPU6050_t* DStrct, _MPU6050_Device_t DeviceNumber, _MPU6050_Accelerometer_t AccelerometerSensitivity, _MPU6050_Gyroscope_t GyroscopeSensitivity) {
	uint8_t temp;
	
	/* Format I2C address */
	DStrct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	
	/* Initialize I2C */
	_I2C_Init(MPU6050_I2C, MPU6050_I2C_PINSPACK, MPU6050_I2C_CLOCK);
	
	/* Check if device is connected */
	if (_I2C_IsDeviceConnected(MPU6050_I2C, DStrct->Address) != _I2C_Result_Ok) {
		/* Return error */
		return _MPU6050_Result_DeviceNotConnected;
	}
	
	/* Check who am I */
	_I2C_Read(MPU6050_I2C, DStrct->Address, MPU6050_WHO_AM_I, &temp);
	if (temp != MPU6050_I_AM) {
		/* Return error */
		return _MPU6050_Result_DeviceInvalid;
	}
	
	/* Wakeup MPU6050 */
	_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_PWR_MGMT_1, 0x00);
	
	/* Set sample rate to 1kHz */
	_MPU6050_SetDataRate(DStrct, _MPU6050_DataRate_1KHz);
	
	/* Config accelerometer */
	_MPU6050_SetAccelerometer(DStrct, AccelerometerSensitivity);
	
	/* Config accelerometer */
	_MPU6050_SetGyroscope(DStrct, GyroscopeSensitivity);
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_SetGyroscope(_MPU6050_t* DStrct, _MPU6050_Gyroscope_t GyroscopeSensitivity) {
	uint8_t temp;
	
	/* Config gyroscope */
	_I2C_Read(MPU6050_I2C, DStrct->Address, MPU6050_GYRO_CONFIG, &temp);
	temp = (temp & 0xE7) | (uint8_t)GyroscopeSensitivity << 3;
	_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_GYRO_CONFIG, temp);
	
	switch (GyroscopeSensitivity) {
		case _MPU6050_Gyroscope_250s:
			DStrct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENSITIVITY_250;
			break;
		case _MPU6050_Gyroscope_500s:
			DStrct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENSITIVITY_500;
			break;
		case _MPU6050_Gyroscope_1000s:
			DStrct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENSITIVITY_1000;
			break;
		case _MPU6050_Gyroscope_2000s:
			DStrct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENSITIVITY_2000;
		default:
			break;
	}
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_SetAccelerometer(_MPU6050_t* DStrct, _MPU6050_Accelerometer_t AccelerometerSensitivity) {
	uint8_t temp;
	
	/* Config accelerometer */
	_I2C_Read(MPU6050_I2C, DStrct->Address, MPU6050_ACCEL_CONFIG, &temp);
	temp = (temp & 0xE7) | (uint8_t)AccelerometerSensitivity << 3;
	_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_ACCEL_CONFIG, temp);
	
	/* Set sensitivities for multiplying gyro and accelerometer data */
	switch (AccelerometerSensitivity) {
		case _MPU6050_Accelerometer_2G:
			DStrct->Acce_Mult = (float)1 / MPU6050_ACCE_SENSITIVITY_2;
			break;
		case _MPU6050_Accelerometer_4G:
			DStrct->Acce_Mult = (float)1 / MPU6050_ACCE_SENSITIVITY_4;
			break;
		case _MPU6050_Accelerometer_8G:
			DStrct->Acce_Mult = (float)1 / MPU6050_ACCE_SENSITIVITY_8;
			break;
		case _MPU6050_Accelerometer_16G:
			DStrct->Acce_Mult = (float)1 / MPU6050_ACCE_SENSITIVITY_16;
		default:
			break;
	}
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_SetDataRate(_MPU6050_t* DStrct, uint8_t rate) {
	/* Set data sample rate */
	if (_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_SMPLRT_DIV, rate) != _I2C_Result_Ok) {
		/* Return error */
		return _MPU6050_Result_Error;
	}
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}
	

_MPU6050_Result_t _MPU6050_EnableInterrupts(_MPU6050_t* DStrct) {
	uint8_t temp;	
	
	/* Enable interrupts for data ready and motion detect */
	_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_INT_ENABLE, 0x21);
	
	/* Clear IRQ flag on any read operation */
	_I2C_Read(MPU6050_I2C, DStrct->Address, MPU6050_INT_PIN_CFG, &temp);
	temp |= 0x10;
	_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_INT_PIN_CFG, temp);
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_DisableInterrupts(_MPU6050_t* DStrct) {
	/* Disable interrupts */
	if (_I2C_Write(MPU6050_I2C, DStrct->Address, MPU6050_INT_ENABLE, 0x00) != _I2C_Result_Ok) {
		/* Return error */
		return _MPU6050_Result_Error;
	}
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_ReadInterrupts(_MPU6050_t* DStrct, _MPU6050_Interrupt_t* InterruptsStruct) {
	uint8_t read;
	
	/* Reset structure */
	InterruptsStruct->Status = 0;
	
	/* Read interrupts status register */
	if (_I2C_Read(MPU6050_I2C, DStrct->Address, MPU6050_INT_STATUS, &read) != _I2C_Result_Ok) {
		/* Return error */
		return _MPU6050_Result_Error;
	}
	
	/* Fill value */
	InterruptsStruct->Status = read;
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_ReadAccelerometer(_MPU6050_t* DStrct) {
	uint8_t acc_buffer[6];
	
	/* Read accelerometer data */
	_I2C_ReadMulti(MPU6050_I2C, DStrct->Address, MPU6050_ACCEL_X_HIGH, acc_buffer, 6);
	
	/* Format */
	DStrct->Accelerometer_x= (int16_t)(acc_buffer[0] << 8 | acc_buffer[1]);
	DStrct->Accelerometer_y = (int16_t)(acc_buffer[2] << 8 | acc_buffer[3]);
	DStrct->Accelerometer_z = (int16_t)(acc_buffer[4] << 8 | acc_buffer[5]);
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_ReadGyroscope(_MPU6050_t* DStrct) {
	uint8_t gyro_buffer[6];
	
	/* Read gyroscope data */
	_I2C_ReadMulti(MPU6050_I2C, DStrct->Address, MPU6050_GYRO_X_HIGH, gyro_buffer, 6);
	
	/* Format */
	DStrct->Gyroscope_x = (int16_t)(gyro_buffer[0] << 8 | gyro_buffer[1]);
	DStrct->Gyroscope_y = (int16_t)(gyro_buffer[2] << 8 | gyro_buffer[3]);
	DStrct->Gyroscope_z = (int16_t)(gyro_buffer[4] << 8 | gyro_buffer[5]);

	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_ReadTemperature(_MPU6050_t* DStrct) {
	uint8_t Temp_buff[2];
	int16_t temp;
	
	/* Read temperature */
	_I2C_ReadMulti(MPU6050_I2C, DStrct->Address, MPU6050_TEMP_OUT_H, Temp_buff, 2);
	
	/* Format temperature */
	temp = (Temp_buff[0] << 8 | Temp_buff[1]);
	DStrct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	
	/* Return OK */
	return _MPU6050_Result_Ok;
}

_MPU6050_Result_t _MPU6050_ReadAll_Data(_MPU6050_t* DStrct) {
	uint8_t R_data[14];
	int16_t temp;
	
	/* Read full raw data, 14bytes */
	_I2C_ReadMulti(MPU6050_I2C, DStrct->Address, MPU6050_ACCEL_X_HIGH, R_data, 14);
	
	/* Format accelerometer data */
	DStrct->Accelerometer_x = (int16_t)(R_data[0] << 8 | R_data[1]);
	DStrct->Accelerometer_y = (int16_t)(R_data[2] << 8 | R_data[3]);
	DStrct->Accelerometer_z = (int16_t)(R_data[4] << 8 | R_data[5]);

	/* Format temperature */
	temp = (R_data[6] << 8 | R_data[7]);
	DStrct->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);
	
	/* Format gyroscope data */
	DStrct->Gyroscope_x = (int16_t)(R_data[8] << 8 | R_data[9]);
	DStrct->Gyroscope_y = (int16_t)(R_data[10] << 8 | R_data[11]);
	DStrct->Gyroscope_z = (int16_t)(R_data[12] << 8 | R_data[13]);

	/* Return OK */
	return _MPU6050_Result_Ok;
}
