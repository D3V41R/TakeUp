#include "main.h"
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mpu6050.h"

uint8_t mpuAddy = 0x68;

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
#define RAD_TO_DEG (180.0f / 3.14159265f)


float phiHat;
float thetaHat;
float phiHat_rad = 0.0f; // roll and pitch angles
float thetaHat_rad = 0.0f;

float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;
#define ALPHA 0.02f




void i2c_write(uint8_t DevAddress, uint8_t *pData, uint16_t Size){


    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress << 1), pData, Size, HAL_MAX_DELAY);
}


void i2c_read(uint8_t DevAddress, uint8_t reg, uint8_t *pData, uint16_t Size){


    HAL_I2C_Master_Transmit(&hi2c1, (DevAddress << 1), &reg, 1, HAL_MAX_DELAY); // write register address to bus

    HAL_I2C_Master_Receive(&hi2c1, (DevAddress << 1), pData, Size, HAL_MAX_DELAY); // read data into pData
}


uint8_t WhoAmI(){

    uint8_t data;

    i2c_read(mpuAddy, 0x75, &data, 1);

    return data; // should return 0x68
}


void sample_rate(){

    uint8_t dlpf[2]  = {0x1A, 0x01}; // DLPF on, 184Hz bandwidth, enables 1kHz gyro rate
    uint8_t smprt[2] = {0x25, 0x00}; // 1000 / (1 + 0) = 1kHz

    i2c_write(mpuAddy, dlpf, 2);
    i2c_write(mpuAddy, smprt, 2);
}


void gyro_config(){

    uint8_t data[2] = {0x1B, 0b00011000}; // +-2000 deg/s

    i2c_write(mpuAddy, data, 2);
}


void accel_config(){

    uint8_t data[2] = {0x1C, 0x00}; // +-16g

    i2c_write(mpuAddy, data, 2);
}


void mpu_init(){

    HAL_Delay(100);

    WhoAmI();

    uint8_t data[2] = {0x6B, 0x00}; // write 0 to power management register
    i2c_write(mpuAddy, data, 2);


    sample_rate();
    gyro_config();
    accel_config();
    calibrate_gyro();
}


void read_Accel(accel_data* acceldata){

    uint8_t data[6]; // each x, y, z is 2 bytes

    i2c_read(mpuAddy, 0x3B, data, 6);

    acceldata->accelX = (int16_t)(data[0] << 8 | data[1]) / 16384.0f;
    acceldata->accelY = (int16_t)(data[2] << 8 | data[3]) / 16384.0f;
    acceldata->accelZ = (int16_t)(data[4] << 8 | data[5]) / 16384.0f;
}


void read_temp(accel_data* acceldata){

    uint8_t data[2];

    i2c_read(mpuAddy, 0x41, data, 2);

    acceldata->temp = (int16_t)(data[0] << 8 | data[1]);
}


void read_gyro(gyro_data* gyrodata){

    uint8_t data[6];

    i2c_read(mpuAddy, 0x43, data, 6);

    gyrodata->gyroX = (int16_t)(data[0] << 8 | data[1]) / 16.4f;
    gyrodata->gyroY = (int16_t)(data[2] << 8 | data[3]) / 16.4f;
    gyrodata->gyroZ = (int16_t)(data[4] << 8 | data[5]) / 16.4f;
}


void log_data(){


    accel_data accel;
    gyro_data gyro;



    char msg[100];


    read_Accel(&accel);
    read_gyro(&gyro);

    // lpf to filter gyro measurements

    filtered_ax = ALPHA * accel.accelX + (1.0f - ALPHA) * filtered_ax;
    filtered_ay = ALPHA * accel.accelY + (1.0f - ALPHA) * filtered_ay;
    filtered_az = ALPHA * accel.accelZ + (1.0f - ALPHA) * filtered_az;

    filtered_gx = ALPHA * gyro.gyroX + (1.0f - ALPHA) * filtered_gx;
    filtered_gy = ALPHA * gyro.gyroY + (1.0f - ALPHA) * filtered_gy;
    filtered_gz = ALPHA * gyro.gyroZ + (1.0f - ALPHA) * filtered_gz;

	float magnitude = sqrtf(filtered_ax * filtered_ax +
			filtered_ay * filtered_ay +
			filtered_az * filtered_az);


	// transforming body rates to eualrs

	float phiHat_rps = gyro.gyroX + tanf(thetaHat_rad) * (sinf(phiHat_rad) * gyro.gyroY + cosf(phiHat_rad) * gyro.gyroZ);
	float thetaHat_rps = cosf(phiHat_rad) * gyro.gyroY - sinf(phiHat_rad) * gyro.gyroZ;

	// getting roll and pitch

	float accel_phi   = atanf(filtered_ay / filtered_az);
	float accel_theta = asinf(filtered_ax / magnitude);


	// integration of eualr rates to get eualar angels , ( OLD + DT[ SAMPLE TIME IN SECONDS] * rate of change)


	phiHat_rad   = phiHat_rad   + phiHat_rps   * 0.001f;
	thetaHat_rad = thetaHat_rad + thetaHat_rps * 0.001f;

	// complementary filter fusion
	phiHat_rad   = 0.98f * phiHat_rad   + 0.05f * accel_phi;
	thetaHat_rad = 0.98f * thetaHat_rad + 0.05f * accel_theta;


//    sprintf(msg, "Pitch:%0.3f, Roll:%0.3f \r\n", phiHat *RAD_TO_DEG , thetaHat *RAD_TO_DEG );
      sprintf(msg, "Pitch:%0.3f, Roll:%0.3f \r\n", phiHat_rad *RAD_TO_DEG , thetaHat_rad *RAD_TO_DEG );

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

void calibrate_gyro(){
    gyro_data gyro;
    float gyroX_bias;
    float gyroY_bias;
    float gyroZ_bias;

    for(int i = 0; i < 1000; i++){
        read_gyro(&gyro);
        gyroX_bias += gyro.gyroX;
        gyroY_bias += gyro.gyroY;
        gyroZ_bias += gyro.gyroZ;
        HAL_Delay(1);
    }
    gyroX_bias /= 1000.0f;
    gyroY_bias /= 1000.0f;
    gyroZ_bias /= 1000.0f;
}


