/*
 * nrf.h
 *
 *  Created on: Sep 2, 2024
 *      Author: VTV02
 */

#ifndef NRF_H_
#define NRF_H_


#include "stm32f103xe.h"
//SPI Pin out
#define NRF24L01_SPI                 SPI1
#define NRF24L01_SPI_GPIO            GPIOA
#define NRF24L01_SPI_SCK_PIN         5
#define NRF24L01_SPI_MISO_PIN        6
#define NRF24L01_SPI_MOSI_PIN        7
#define NRF24L01_CE_PIN              4
#define NRF24L01_CSN_PIN             3



// NRF24L01 commands
#define NRF24L01_CMD_R_REGISTER          0x00
#define NRF24L01_CMD_W_REGISTER          0x20
#define NRF24L01_CMD_R_RX_PAYLOAD        0x61
#define NRF24L01_CMD_W_TX_PAYLOAD        0xA0
#define NRF24L01_CMD_FLUSH_TX            0xE1
#define NRF24L01_CMD_FLUSH_RX            0xE2
#define NRF24L01_CMD_REUSE_TX_PL         0xE3
#define NRF24L01_CMD_NOP                 0xFF

// NRF24L01 registers
#define NRF24L01_REG_CONFIG              0x00
#define NRF24L01_REG_EN_AA               0x01
#define NRF24L01_REG_EN_RXADDR           0x02
#define NRF24L01_REG_SETUP_AW            0x03
#define NRF24L01_REG_SETUP_RETR          0x04
#define NRF24L01_REG_RF_CH               0x05
#define NRF24L01_REG_RF_SETUP            0x06
#define NRF24L01_REG_STATUS              0x07
#define NRF24L01_REG_RX_ADDR_P0          0x0A
#define NRF24L01_REG_TX_ADDR             0x10
#define NRF24L01_REG_RX_PW_P0            0x11
#define NRF24L01_REG_FIFO_STATUS         0x17

// Function prototypes
void NRF24L01_Init(void);
void NRF24L01_WriteReg(uint8_t reg, uint8_t value);
uint8_t NRF24L01_ReadReg(uint8_t reg);
void NRF24L01_WriteTxPayload(uint8_t *data, uint8_t length);
void NRF24L01_ReadRxPayload(uint8_t *data, uint8_t length);
void NRF24L01_FlushTx(void);
void NRF24L01_FlushRx(void);



#endif /* NRF_H_ */
