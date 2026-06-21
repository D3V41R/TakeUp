#include "Motors.h"



extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

void Motor_Init(void){	// arm ESC, wait for beeps

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    TIM2->CCR1 = 2000;    // full throttle
    HAL_Delay(2000);
    TIM2->CCR1 = 1000;    //minimum throttle
    HAL_Delay(2000);      // wait for beeps to finish
}


void Motor_SetSpeed(uint8_t percent){ // 0-100% mapped to 1000-2000
	uint16_t CCR = 1000 + (percent * 10);
	TIM2->CCR1 = CCR;


}


void Motor_Stop(void){
    TIM2->CCR1 = 1000;
}



void Motor_SetRaw(uint16_t ccr){
	TIM2->CCR1 = ccr;

}

void Motor_Control_UART(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t len){
    HAL_UART_Receive(huart, buff, len, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, buff, len, HAL_MAX_DELAY); // echo back what you received

    if (buff[0] == 'S') {
        uint8_t percent = atoi((char*)&buff[1]);
        Motor_SetSpeed(percent);
        HAL_UART_Transmit(huart, (uint8_t*)"Speed Set\r\n", 11, HAL_MAX_DELAY);
    } else if (buff[0] == 'R') {
        uint16_t ccr = atoi((char*)&buff[1]);
        Motor_SetRaw(ccr);
        HAL_UART_Transmit(huart, (uint8_t*)"Raw Set\r\n", 9, HAL_MAX_DELAY);
    } else if (buff[0] == 'X') {
        Motor_Stop();
        HAL_UART_Transmit(huart, (uint8_t*)"Stopped\r\n", 9, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(huart, (uint8_t*)"Unknown CMD\r\n", 13, HAL_MAX_DELAY);
    }
}
