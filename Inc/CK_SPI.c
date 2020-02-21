
#include "stm32f4xx.h"
#include "CK_SPI.h"
#include "CK_GPIO.h"

uint32_t timeout;

int isSPIx_Initialized[6] = {0,0,0,0,0,0};//0->SPI1, 1->SPI2 ...

void CK_SPI_Init(SPI_TypeDef* spi_n){

	SPI_TypeDef* SPIx = spi_n;
	GPIO_TypeDef* GPIOx;
	CK_GPIOx_AFx AFx;
	uint16_t miso_pin, mosi_pin, sck_pin;

	/*
	 *	SPI1,4,5 have APB2=90MHz clock so prescaler must be selected
	 * 	To meet 10MHZ max spi clock speed
	 *
	 */

	if(SPIx == SPI1){
		GPIOx = GPIOA;
		sck_pin = 5;
		miso_pin = 6;
		mosi_pin = 7;
		AFx = CK_GPIO_AF5;
		RCC->APB2ENR |= CK_RCC_SPI1_EN;//Enable related SPI clock
		isSPIx_Initialized[0] = 1;
	}
	else if(SPIx == SPI2){
		GPIOx = GPIOB;
		sck_pin  = 13;
		miso_pin = 14;
		mosi_pin = 15;
		AFx = CK_GPIO_AF5;
		RCC->APB1ENR |= CK_RCC_SPI2_EN;//Enable related SPI clock
		isSPIx_Initialized[1] = 1;
	}

	CK_GPIOx_ClockEnable(GPIOx);
	CK_GPIOx_Init(GPIOx, sck_pin,  CK_GPIO_AF, AFx, CK_GPIO_PUSHPULL, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIOx_Init(GPIOx, miso_pin, CK_GPIO_AF, AFx, CK_GPIO_PUSHPULL, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);
	CK_GPIOx_Init(GPIOx, mosi_pin, CK_GPIO_AF, AFx, CK_GPIO_PUSHPULL, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	/*
	 * Default: full duplex, (cpha=0,cpol=0),8bit data,No CRC,MSB First
	 */
	if(SPIx == SPI2){
		SPIx->CR1 |= CK_SPIx_CR1_MSTR | CK_SPIx_CR1_SSM | CK_SPIx_CR1_SSI | CK_SPIx_CR1_Fclk_Div4;
		SPIx->CR1 |= CK_SPIx_CR1_SPE;//SPI Enable
	}
	else{
		SPIx->CR1 |= CK_SPIx_CR1_MSTR | CK_SPIx_CR1_SSM | CK_SPIx_CR1_SSI | CK_SPIx_CR1_Fclk_Div8;
		SPIx->CR1 |= CK_SPIx_CR1_SPE;//SPI Enable
	}



}

uint8_t CK_SPI_WriteRegister(uint8_t reg, uint8_t data, SPI_TypeDef* SPIn, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin){
	uint8_t val = 0;
	CK_GPIOx_ClearPin(GPIOx_CS, cs_pin);

	CK_SPI_Transfer(SPIn, reg);
	val = CK_SPI_Transfer(SPIn, data);

	CK_GPIOx_SetPin(GPIOx_CS, cs_pin);
	return val;
}

uint8_t CK_SPI_Transfer(SPI_TypeDef* SPIn, uint8_t data){

	timeout = SPI_TIMEOUT;
	while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
		if(--timeout == 0x00){return 1;}
	}
	SPIn->DR = data;

	timeout = SPI_TIMEOUT;
	while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
		if(--timeout == 0x00){return 1;}
	}
	return SPIn->DR;
}

void CK_SPI_ReadRegisterMulti(uint8_t reg, SPI_TypeDef* SPIn, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin, uint8_t* dataIn, int count){

	CK_GPIOx_ClearPin(GPIOx_CS, cs_pin);

	CK_SPI_Transfer(SPIn, reg | 0x80);

	while (count--) {

		*dataIn++ = CK_SPI_Transfer(SPIn, 0);
	}

	CK_GPIOx_SetPin(GPIOx_CS, cs_pin);

}

uint8_t CK_SPI_WriteMulti(SPI_TypeDef* SPIn, uint8_t* dataOut, uint32_t count){

	while (count--) {
		timeout = SPI_TIMEOUT;
		while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
			if(--timeout == 0x00){return 1;}
		}

		*(__IO uint8_t *)&SPIn->DR = *dataOut++;

		timeout = SPI_TIMEOUT;
		while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
			if(--timeout == 0x00){return 1;}
		}

		(void)*(__IO uint16_t *)&SPIn->DR;

	}
	return 0;
}

uint8_t CK_SPI_ReadMulti(SPI_TypeDef* SPIn, uint8_t* dataIn, uint8_t dummy, uint32_t count){

	while (count--) {
		timeout = SPI_TIMEOUT;
		while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
			if(--timeout == 0x00){return 1;}
		}

		*(__IO uint8_t *)&SPIn->DR = dummy;

		timeout = SPI_TIMEOUT;
		while(((SPIn)->SR & (CK_SPIx_SR_TXE | CK_SPIx_SR_RXNE)) == 0 || ((SPIn)->SR & CK_SPIx_SR_BSY)){
			if(--timeout == 0x00){return 1;}
		}

		*dataIn++ = *(__IO uint8_t *)&SPIn->DR;
	}
	return 0;
}

int CK_SPI_isSPIxInitialized(int n){
	if(n < 1 || n > 5){// Invalid
		return 2;
	}
	return isSPIx_Initialized[n-1];
}
