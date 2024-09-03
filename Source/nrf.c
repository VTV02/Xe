#include "nrf.h"

static void NRF24L01_LowLevelInit(void)
{
    // Enable clock for GPIOA and SPI1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_SPI1EN;

    // Configure SPI pins
    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    GPIOA->CRL |= GPIO_CRL_MODE5_1 | GPIO_CRL_CNF5_1 | GPIO_CRL_MODE7_1 | GPIO_CRL_CNF7_1;

    GPIOA->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6);
    GPIOA->CRL |= GPIO_CRL_CNF6_0;

    // Configure CE and CSN pins
    GPIOA->CRL &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3 | GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    GPIOA->CRL |= GPIO_CRL_MODE3_1 | GPIO_CRL_MODE4_1;

    // Configure SPI
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR2 = 0;
    SPI1->CR1 |= SPI_CR1_SPE;
}

static void SPI_Init()
{
	// Bật clock cho SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	/*Set baud rate control F/32*/
	SPI1->CR1&=~(1U<<3);
	SPI1->CR1&=~(1U<<4);
	SPI1->CR1|=(1U<<5);


	/*Set CPHA =0, CPOL =0*/
	SPI1->CR1&=~(1U<<0);
	SPI1->CR1&=~(1U<<1);

	/*Set DFF(data frame format) 8 bit*/
	SPI1->CR1&=~(1U<<11);
	SPI1->CR1|=(1U<<8);
	SPI1->CR1|=(1U<<9);

	/*Configure MSBFIRST*/
	SPI1->CR1&=~(1U<<7);

	/*Set MSTR */
	SPI1->CR1|=(1U<<2);

}
static uint8_t NRF24L01_SendByte(uint8_t byte)
{
    while (!(SPI1->SR & SPI_SR_TXE));
    SPI1->DR = byte;
    while (!(SPI1->SR & SPI_SR_RXNE));
    return SPI1->DR;
}

void NRF24L01_Init(void)
{
    NRF24L01_LowLevelInit();
    SPI_Init();

    // Set initial CE and CSN states
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);// CSN to High
    GPIOA->BRR = (1 << NRF24L01_CE_PIN);//CE to low

    // Configure NRF24L01
    NRF24L01_WriteReg(NRF24L01_REG_CONFIG, 0x0A);  // Enable CRC (2 bytes) & Power Up
    NRF24L01_WriteReg(NRF24L01_REG_EN_AA, 0x01);   // Enable Auto-Ack on pipe 0
    NRF24L01_WriteReg(NRF24L01_REG_EN_RXADDR, 0x01);  // Enable RX address on pipe 0
    NRF24L01_WriteReg(NRF24L01_REG_SETUP_AW, 0x03);   // Set address width to 5 bytes
    NRF24L01_WriteReg(NRF24L01_REG_SETUP_RETR, 0x5F); // 1500us retry delay, 15 retries
    NRF24L01_WriteReg(NRF24L01_REG_RF_CH, 0x02);      // Set RF channel to 2
    NRF24L01_WriteReg(NRF24L01_REG_RF_SETUP, 0x0E);   // 2Mbps, 0dBm power
    NRF24L01_WriteReg(NRF24L01_REG_RX_PW_P0, 0x20);   // 32 byte payload on pipe 0
}

void NRF24L01_WriteReg(uint8_t reg, uint8_t value)
{

    GPIOA->BRR = (1 << NRF24L01_CSN_PIN);
    NRF24L01_SendByte(NRF24L01_CMD_W_REGISTER | reg);
    NRF24L01_SendByte(value);
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
    uint8_t value;
    GPIOA->BRR = (1 << NRF24L01_CSN_PIN);
    NRF24L01_SendByte(NRF24L01_CMD_R_REGISTER | reg);
    value = NRF24L01_SendByte(NRF24L01_CMD_NOP);
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);
    return value;
}

void NRF24L01_WriteTxPayload(uint8_t *data, uint8_t length)
{
    GPIOA->BRR = (1 << NRF24L01_CSN_PIN);
    NRF24L01_SendByte(NRF24L01_CMD_W_TX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        NRF24L01_SendByte(data[i]);
    }
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);
}

void NRF24L01_ReadRxPayload(uint8_t *data, uint8_t length)
{
    GPIOA->BRR = (1 << NRF24L01_CSN_PIN);
    NRF24L01_SendByte(NRF24L01_CMD_R_RX_PAYLOAD);
    for (uint8_t i = 0; i < length; i++) {
        data[i] = NRF24L01_SendByte(NRF24L01_CMD_NOP);
    }
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);
}

void NRF24L01_FlushTx(void)
{
    GPIOA->BRR = (1 << NRF24L01_CSN_PIN);
    NRF24L01_SendByte(NRF24L01_CMD_FLUSH_TX);
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);
}

void NRF24L01_FlushRx(void)
{
    GPIOA->BRR = (1 << NRF24L01_CSN_PIN);
    NRF24L01_SendByte(NRF24L01_CMD_FLUSH_RX);
    GPIOA->BSRR = (1 << NRF24L01_CSN_PIN);
}
