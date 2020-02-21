
#ifndef CK_SPI_H_
#define CK_SPI_H_

#define CK_SPIx_CR1_MSTR               1u<<2
#define CK_SPIx_CR1_CPOL               1u<<1
#define CK_SPIx_CR1_CPHA               1u<<0
#define CK_SPIx_CR1_SPE                1u<<6
#define CK_SPIx_CR1_SSI                1u<<8
#define CK_SPIx_CR1_SSM                1u<<9

#define CK_SPIx_SR_TXE                 1u<<1
#define CK_SPIx_SR_RXNE                1u<<0
#define CK_SPIx_SR_BSY                 1u<<7

#define CK_RCC_SPI1_EN                 1u<<12
#define CK_RCC_SPI2_EN                 1u<<14
#define CK_RCC_SPI4_EN                 1u<<13
#define CK_RCC_SPI5_EN                 1u<<20
#define CK_RCC_SPI6_EN                 1u<<21

#define SPI_TIMEOUT	100000

typedef enum
{
	CK_SPIx_CR1_Fclk_Div2                 = 0u<<3,
	CK_SPIx_CR1_Fclk_Div4                 = 1u<<3,
	CK_SPIx_CR1_Fclk_Div8                 = 2u<<3,
	CK_SPIx_CR1_Fclk_Div16                = 3u<<3,
	CK_SPIx_CR1_Fclk_Div32                = 4u<<3,
	CK_SPIx_CR1_Fclk_Div64                = 5u<<3,
	CK_SPIx_CR1_Fclk_Div128               = 6u<<3,
	CK_SPIx_CR1_Fclk_Div256               = 7u<<3

}CK_SPIx_CR1_Fclk_Div;

void CK_SPI_Init(SPI_TypeDef* spi_n);

uint8_t CK_SPI_WriteRegister(uint8_t reg, uint8_t data, SPI_TypeDef* SPIn, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin);

uint8_t CK_SPI_Transfer(SPI_TypeDef* SPIn, uint8_t data);

void CK_SPI_ReadRegisterMulti(uint8_t reg, SPI_TypeDef* SPIn, GPIO_TypeDef* GPIOx_CS, uint16_t cs_pin, uint8_t* dataIn, int count);

int CK_SPI_isSPIxInitialized(int n);

// FatFS uses
uint8_t CK_SPI_WriteMulti(SPI_TypeDef* SPIn, uint8_t* dataOut, uint32_t count);

// FatFS uses
uint8_t CK_SPI_ReadMulti(SPI_TypeDef* SPIn, uint8_t* dataIn, uint8_t dummy, uint32_t count);



#endif /* CK_SPI_H_ */
