/*
 * mpu6050.h
 *
 *  Created on: Jun 11, 2026
 *      Author: Zahir
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>

typedef struct{ float accelX; float accelY; float accelZ; float temp; } accel_data;
typedef struct{ float gyroX; float gyroY; float gyroZ; } gyro_data;


void mpu_init(void);
void log_data(void);
void read_Accel(accel_data* acceldata);
void read_gyro(gyro_data* gyrodata);
void read_temp(accel_data* acceldata);
void calibrate_gyro(void);

#endif /* INC_MPU6050_H_ */
