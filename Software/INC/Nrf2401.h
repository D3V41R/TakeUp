

#ifndef INC_NRF2401_H_
#define INC_NRF2401_H_


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "math.h"


void spi_transmit(uint8_t* data, uint32_t size);
void spi_recieve(uint8_t* data, uint32_t size);
void Nrf_init();
void Nrf_pwr();
void Csn_Enable();
void read_status();
void Rf_setup();
void config_channel();
void Transmit_nrf(uint8_t* payload, uint8_t size);
uint8_t Recieve_nrf(uint8_t* payload, uint8_t size);
void enable_auto_ack();
void enable_rx_pipe();
void set_retries();


#endif /* INC_NRF2401_H_ */
