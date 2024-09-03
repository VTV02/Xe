#include "nrf.h"



int main()
{

	// Kích hoạt clock cho GPIOB (nếu chưa kích hoạt)
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

		// Cấu hình PB1 là output push-pull
		GPIOB->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1); // Xóa các bits MODE1 và CNF1
		GPIOB->CRL |= GPIO_CRL_MODE1_1; // MODE1 = 10 (Output mode, max speed 2 MHz)
		                                 // CNF1 = 00 (General purpose output push-pull
	NRF24L01_Init();

	while(1)
	{}

}
void ReceiveData() {
    uint8_t data;
    // Kiểm tra xem có dữ liệu trong RX FIFO không
    if (NRF24L01_ReadReg(NRF24L01_REG_STATUS) & (1 << 6)) {  // Kiểm tra bit RX_DR (Data Ready)
        NRF24L01_ReadRxPayload(&data, 1);  // Đọc dữ liệu từ RX FIFO
        NRF24L01_WriteReg(NRF24L01_REG_STATUS, (1 << 6));  // Xóa cờ RX_DR

        // Kiểm tra nếu dữ liệu nhận được là số 1
        if (data == 1) {
            GPIOB->BSRR = (1 << 0);  // Giả sử LED nối với chân PA1
        }
    }
}

