#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "main.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"

// CCR limits
#define MOTOR_MIN  1000
#define MOTOR_MAX  2000
#define MOTOR_ARM  1000

void Motor_Init(void);       // arm ESC, wait for beeps
void Motor_SetSpeed(uint8_t percent);  // 0-100% mapped to 1000-2000
void Motor_Stop(void);       // set back to 1000
void Motor_SetRaw(uint16_t ccr);  // direct CCR control for testing
void Motor_Control_UART(UART_HandleTypeDef *huart, uint8_t *buff, uint16_t len);

#endif
