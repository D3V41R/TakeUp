#include "Dshot150.h"
#include "main.h"
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;

typedef struct {
	uint16_t throttle;
	uint8_t telemetry;
	uint16_t frame;
}Dshot150;

uint16_t dshotbuffer[17];

void readBits(uint16_t data) {
    for (int i = 0; i < 16; i++) {
        uint16_t bitVal = (data >> (15 - i)) & 0x1;
        dshotbuffer[i] = bitVal ? 170 : 85;
    }
    dshotbuffer[16] = 0;
}

uint8_t calculateCRC(uint16_t data) {
	uint16_t CRCC;
	CRCC = (data ^ (data >> 4) ^ (data >> 8)) & 0xF;
	return CRCC;
}

void buildDShotFrame(Dshot150* motor, uint16_t throttleVal, uint8_t telemetryVal) {
	uint16_t data;
	uint8_t crc;
	motor -> throttle = throttleVal;
	motor -> telemetry = telemetryVal;
	data = (motor->throttle << 1) | motor -> telemetry;
	crc = calculateCRC(data);
	motor -> frame = (data << 4) | crc;
}

void pwm_init(){
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*) dshotbuffer, 17);
}

volatile uint8_t dshot_ready = 1;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
        dshot_ready = 1;
    }
}

void sendThrottle(Dshot150 *motor, uint16_t throttleVal, uint8_t telemetryVal) {
    if (!dshot_ready) return;
    dshot_ready = 0;
    buildDShotFrame(motor, throttleVal, telemetryVal);
    readBits(motor->frame);
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)dshotbuffer, 17);
}


Dshot150 testMotor;

void dshot_test_loop(void) {
    while (1) {
        buildDShotFrame(&testMotor, 1000, 0); // create data frame
        readBits(testMotor.frame); // read each bit in the data frame if its 0 or 1

        HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t*)dshotbuffer, 17); // send the data via DMA
        HAL_Delay(2);
        HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);

        HAL_Delay(2);
    }
}
