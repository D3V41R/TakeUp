#include "main.h"
#include "Nrf2401.h"
#include <string.h>


// make sure to write it so that it transmits the spi data over uart bro

extern SPI_HandleTypeDef hspi2;
extern UART_HandleTypeDef huart2;


void spi_transmit(uint8_t* data, uint32_t size){
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET); // setting csn low
	HAL_SPI_Transmit(&hspi2, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET); // setting csn hi

}

void spi_recieve(uint8_t* data, uint32_t size){
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET); // settign csn low
	HAL_SPI_Receive(&hspi2, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET); // setting csn hi
}


void spi_transmit_receive(uint8_t* tx, uint8_t* rx, uint32_t size){
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}

void Nrf_pwr_on(){

	// 0x20 = write command
	// 0x00 = read command

	// sending pwr on command

	HAL_Delay(11); // delay 11 ms before
	uint8_t data[2] = {0x20 | 0x00, 0x0F}; // rx mode + powE ruop
	spi_transmit(data, 2);
	HAL_Delay(2); // delay 2ms before entering standbymode

	// in standby mode

}

void read_status()
{
    uint8_t tx = 0xFF;  // NOP returns STATUS
    uint8_t rx = 0;
    char buf[30];

    spi_transmit_receive(&tx, &rx, 1);

    sprintf(buf, "STATUS:0x%02X\r\n", rx);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 1000);
}

void Rf_setup(){
    // 1Mbps, 0dBm
	uint8_t setup_config[2] = {0x20 | 0x06, 0x03};
    spi_transmit(setup_config, 2);

    // channel 85
    uint8_t channel_frq[2] = {0x20 | 0x05, 0x55};
    spi_transmit(channel_frq, 2);

    uint8_t addr_width[2] = {0x20 | 0x03, 0x03};
    spi_transmit(addr_width, 2);

    // RX address = all zeros (match ESP32)
    uint8_t rx_addr[6] = {0x20 | 0x0A, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    spi_transmit(rx_addr, 6);


    // payload size = 16
    uint8_t data[2] = {0x20 | 0x11, 0x10};
    spi_transmit(data, 2);

    // disable auto ack
    uint8_t no_ack[2] = {0x20 | 0x01, 0x00};
    spi_transmit(no_ack, 2);
}



void enable_rx_pipe(){
    uint8_t data[2] = {0x20 | 0x02, 0x01}; // enable pipe 0
    spi_transmit(data, 2);
}

void set_retries(){
    uint8_t data[2] = {0x20 | 0x04, 0x3F}; // 1ms delay, 15 retries
    spi_transmit(data, 2);
}

void Transmit_nrf(uint8_t* payload, uint8_t size){
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);// setting ce low

    // flush TX buffer
    uint8_t flush_tx = 0xE1;
    spi_transmit(&flush_tx, 1);


    // write payload
    uint8_t cmd = 0xA0; // W_TX_PAYLOAD command
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET); // chip seelct
    HAL_SPI_Transmit(&hspi2, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi2, payload, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

    // pulse CE high to trigger transmission
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

    // poll status until TX_DS or MAX_RT
    uint8_t tx = 0x07;
    uint8_t status = 0;
    char buff[40];

    while(1){
        spi_transmit_receive(&tx, &status, 1);

        if(status & (1 << 5)){  // TX_DS = success
            sprintf(buff, "TX OK STATUS:0x%02X\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
            break;
        } else if(status & (1 << 4)){  // MAX_RT = failed
            sprintf(buff, "TX FAIL STATUS:0x%02X\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
            break;
        } else {
            sprintf(buff, "TX PENDING STATUS:0x%02X\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
        }
    }

}

void enable_auto_ack(){
    uint8_t data[2] = {0x20 | 0x01, 0x01}; // enable auto ack
    spi_transmit(data, 2);
}

uint8_t Recieve_nrf(uint8_t* payload, uint8_t size)
{
    uint8_t tx = 0xFF;
    uint8_t status = 0;
    char buff[80];

    uint32_t timeout = HAL_GetTick();

    while (1)
    {
        spi_transmit_receive(&tx, &status, 1);

        if (status & (1 << 6)) // RX_DR
        {
            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

            uint8_t cmd = 0x61; // R_RX_PAYLOAD

            HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
            HAL_SPI_Transmit(&hspi2, &cmd, 1, 1000);
            HAL_SPI_Receive(&hspi2, payload, size, 1000);
            HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

            // clear RX_DR
            uint8_t clear[2] = {0x20 | 0x07, 0x40};
            spi_transmit(clear, 2);

            HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);

            return 1; // got packet
        }

        if (HAL_GetTick() - timeout > 1000)
        {
            sprintf(buff, "RX TIMEOUT S:0x%02X\r\n", status);
            HAL_UART_Transmit(&huart2, (uint8_t*)buff, strlen(buff), 1000);
            return 0; // no packet
        }
    }
}
void Nrf_init(){
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    Nrf_pwr_on();
    Rf_setup();
    enable_rx_pipe();

    // clear RX_DR, TX_DS, MAX_RT
    uint8_t clear_flags[2] = {0x20 | 0x07, 0x70};
    spi_transmit(clear_flags, 2);

    // flush RX FIFO
    uint8_t flush_rx = 0xE2;
    spi_transmit(&flush_rx, 1);

    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET); // start listening
    HAL_Delay(2);
}


