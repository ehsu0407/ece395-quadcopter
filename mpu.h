#include "type.h"
#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDRESS	0xD0

#define MPU6050_RA_SMPLRT_DIV 			0x19
#define MPU6050_RA_CONFIG 				0x1A
#define MPU6050_RA_GYRO_CONFIG 			0x1B
#define MPU6050_RA_ACCEL_CONFIG 		0x1C
#define MPU6050_RA_ACCEL_XOUT_H 		0x3B
#define MPU6050_RA_ACCEL_XOUT_L 		0x3C
#define MPU6050_RA_ACCEL_YOUT_H 		0x3D
#define MPU6050_RA_ACCEL_YOUT_L 		0x3E
#define MPU6050_RA_ACCEL_ZOUT_H 		0x3F
#define MPU6050_RA_ACCEL_ZOUT_L 		0x40
#define MPU6050_RA_TEMP_OUT_H 			0x41
#define MPU6050_RA_TEMP_OUT_L 			0x42
#define MPU6050_RA_GYRO_XOUT_H 			0x43
#define MPU6050_RA_GYRO_XOUT_L 			0x44
#define MPU6050_RA_GYRO_YOUT_H 			0x45
#define MPU6050_RA_GYRO_YOUT_L 			0x46
#define MPU6050_RA_GYRO_ZOUT_H 			0x47
#define MPU6050_RA_GYRO_ZOUT_L 			0x48
#define MPU6050_RA_USER_CTRL 			0x6A
#define MPU6050_RA_PWR_MGMT_1 			0x6B
#define MPU6050_RA_PWR_MGMT_2 			0x6C
#define MPU6050_RA_WHO_AM_I 			0x75



uint8_t 	MPU6050_init(void);
uint8_t 	MPU6050_whoami(void);
int16_t		MPU6050_getGyroX_raw(void);
float   	MPU6050_getGyroX_degree(void);
int16_t		MPU6050_getGyroY_raw(void);
float   	MPU6050_getGyroY_degree(void);
int16_t		MPU6050_getGyroZ_raw(void);
float   	MPU6050_getGyroZ_degree(void);
int16_t 	MPU6050_getAccel_x_raw(void);
int16_t 	MPU6050_getAccel_y_raw(void);
int16_t 	MPU6050_getAccel_z_raw(void);
float 		MPU6050_getAccel_x(void);
float 		MPU6050_getAccel_y(void);
float 		MPU6050_getAccel_z(void);
void    	MPU6050_setZero(void);

#endif /* MPU6050_H_ */
