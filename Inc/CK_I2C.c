
#include "CK_I2C.h"
#include "CK_GPIO.h"
#include "CK_TIME_HAL.h"

I2C_TypeDef* I2Cx;
uint32_t timeout;

int isI2Cx_Initialized[6] = {0,0,0,0,0,0};//0->I2C1, 1->I2C2 ...

void CK_I2C_Init(I2C_TypeDef* i2c, CK_I2C_Speed freq){

	I2Cx = i2c;
	if(I2Cx == I2C1){

		RCC->APB1ENR |= CK_RCC_APB1ENR_I2C1_ENABLE;
		CK_GPIOx_ClockEnable(GPIOB);
		CK_GPIOx_Init(GPIOB,6,CK_GPIO_AF,CK_GPIO_AF4,CK_GPIO_OPENDRAIN,CK_GPIO_HIGH,CK_GPIO_PULLUP);//SCL
		CK_GPIOx_Init(GPIOB,7,CK_GPIO_AF,CK_GPIO_AF4,CK_GPIO_OPENDRAIN,CK_GPIO_HIGH,CK_GPIO_PULLUP);//SDA
		isI2Cx_Initialized[0] = 1;
	}
	else if(I2Cx == I2C2){

		RCC->APB1ENR |= CK_RCC_APB1ENR_I2C2_ENABLE;
		CK_GPIOx_ClockEnable(GPIOB);
		CK_GPIOx_Init(GPIOB,10,CK_GPIO_AF,CK_GPIO_AF4,CK_GPIO_OPENDRAIN,CK_GPIO_HIGH,CK_GPIO_PULLUP);//SCL
		CK_GPIOx_Init(GPIOB,11,CK_GPIO_AF,CK_GPIO_AF4,CK_GPIO_OPENDRAIN,CK_GPIO_HIGH,CK_GPIO_PULLUP);//SDA
		isI2Cx_Initialized[1] = 1;
	}
	else if(I2Cx == I2C3){

		RCC->APB1ENR |= CK_RCC_APB1ENR_I2C3_ENABLE;
		CK_GPIOx_ClockEnable(GPIOB);
		CK_GPIOx_ClockEnable(GPIOC);
		CK_GPIOx_Init(GPIOA,8,CK_GPIO_AF,CK_GPIO_AF4,CK_GPIO_OPENDRAIN,CK_GPIO_HIGH,CK_GPIO_PULLUP);//SCL
		CK_GPIOx_Init(GPIOC,9,CK_GPIO_AF,CK_GPIO_AF4,CK_GPIO_OPENDRAIN,CK_GPIO_HIGH,CK_GPIO_PULLUP);//SDA
		isI2Cx_Initialized[2] = 1;
	}

	uint8_t cr2_freq,trise_val;//6 bit values
	uint16_t ccr_val;//12 bit value

	if(freq==CK_I2C_100Khz){

		cr2_freq = HAL_RCC_GetPCLK1Freq()/1000000;//45MHz or 42MHz;
		ccr_val = ((cr2_freq*1000000)/100000)/2;//2*CCR*(1/45MHz) = 1/100000
		//trise_val = (1000*cr2_freq)/1000;//1000ns max rise time selected for slow mode
		trise_val = cr2_freq+1;

		I2Cx->CR1 &= ~CK_I2C_CR1_PE;//Peripheral Disable
		I2Cx->CR2 = cr2_freq;
		I2Cx->CCR = ccr_val;
		I2Cx->TRISE = trise_val;
		I2Cx->CR1 |= CK_I2C_CR1_PE;//Peripheral Enable
		I2Cx->OAR1 |= 1u<<14;

	}
	else if(freq==CK_I2C_400Khz){

		cr2_freq = HAL_RCC_GetPCLK1Freq()/1000000;//45MHz or 42MHz;
		ccr_val = ((cr2_freq*1000000)/400000)/25;//25*CCR*(1/45MHz) = 1/400000
		trise_val = (((300*cr2_freq)/1000)+1);//300ns max rise time selected for fast mode

		I2Cx->CR1 &= ~CK_I2C_CR1_PE;//Peripheral Disable
		I2Cx->CR2 = cr2_freq;
		I2Cx->CCR = CK_I2C_CCR_FM | CK_I2C_CCR_DUTY | ccr_val;
		I2Cx->TRISE = trise_val;
		I2Cx->CR1 |= CK_I2C_CR1_PE;//Peripheral Enable
		I2Cx->OAR1 |= 1u<<14;
	}
}

void CK_I2C_Transfer(uint8_t slaveAddress, uint8_t reg, uint8_t data){

	CK_I2C_START(slaveAddress,CK_I2C_Transmit,CK_I2C_ACKDisable);

	timeout = TIMEOUT;
	while((I2Cx->SR1 & CK_I2C_SR1_TXE) == 0){
		if(--timeout == 0x00){return;}
	}
	I2Cx->DR = reg;

	timeout = TIMEOUT;
	while((I2Cx->SR1 & CK_I2C_SR1_TXE) == 0){
		if(--timeout == 0x00){return;}
	}
	I2Cx->DR = data;

	CK_I2C_STOP();
}

void CK_I2C_ReadMulti(uint8_t slaveAddress, uint8_t reg, uint8_t* dataStore, int quantity){

	CK_I2C_START(slaveAddress,CK_I2C_Transmit,CK_I2C_ACKEnable);

	timeout = TIMEOUT;
	while((I2Cx->SR1 & CK_I2C_SR1_TXE) == 0){
		if(--timeout == 0x00){return;}
	}
	I2Cx->DR = reg;

	CK_I2C_START(slaveAddress,CK_I2C_Receive,CK_I2C_ACKEnable);

	while(quantity--){
		if(!quantity){
			/* Last byte */
			*dataStore++ = CK_I2C_ReadNack(I2Cx);
		}
		else{
			*dataStore++ = CK_I2C_ReadAck(I2Cx);
		}
	}
}

uint8_t CK_I2C_ReadAck(I2C_TypeDef* I2Cx) {
	uint8_t data;

	/* Enable ACK */
	I2Cx->CR1 |= CK_I2C_CR1_ACK;

	/* Wait till not received */
	timeout = TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if(--timeout == 0x00){return 1;}
	}

	/* Read data */
	data = I2Cx->DR;

	/* Return data */
	return data;
}

uint8_t CK_I2C_ReadNack(I2C_TypeDef* I2Cx) {
	uint8_t data;

	/* Disable ACK */
	I2Cx->CR1 &= ~I2C_CR1_ACK;

	/* Generate stop */
	I2Cx->CR1 |= I2C_CR1_STOP;

	/* Wait till received */

	timeout = TIMEOUT;
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)){
		if(--timeout == 0x00){return 1;}
	}

	/* Read data */
	data = I2Cx->DR;

	/* Return data */
	return data;
}

void CK_I2C_START(uint8_t slaveAddress,CK_I2C_Mode mode,CK_I2C_ACK_Mode ack){

	I2Cx->CR1 |= I2C_CR1_START;

	/* Wait till I2C is busy */
	timeout = TIMEOUT;
	while (!(I2Cx->SR1 & I2C_SR1_SB)){
		if(--timeout == 0x00){return;}
	}

	/* Enable ack if we select it */
	if (ack) {
		I2Cx->CR1 |= I2C_CR1_ACK;
	}

	/* Send write/read bit */
	if (mode == CK_I2C_Transmit) {
		/* Send address with zero last bit */
		I2Cx->DR = slaveAddress<<1;

		/* Wait till finished */
		timeout = TIMEOUT;
		while (!(I2Cx->SR1 & I2C_SR1_ADDR)){
			if(--timeout == 0x00){return;}
		}
	}
	if (mode == CK_I2C_Receive) {
		/* Send address with 1 last bit */
		I2Cx->DR = (slaveAddress<<1) | 1u<<0;

		/* Wait till finished */

		timeout = TIMEOUT;
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
			if(--timeout == 0x00){return;}
		}
	}

	/* Read status register to clear ADDR flag */
	I2Cx->SR2;

}

void CK_I2C_STOP(void){
	timeout = TIMEOUT;
	while(((I2Cx->SR1 & CK_I2C_SR1_TXE) == 0) || ((I2Cx->SR1 & CK_I2C_SR1_BTF) == 0)){
		if(--timeout == 0x00){return;}
	}
	I2Cx->CR1 |= CK_I2C_CR1_STOP;
}


ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
  uint32_t lastevent = 0;
  uint32_t flag1 = 0, flag2 = 0;
  ErrorStatus status = ERROR;

  /* Read the I2Cx status register */
  flag1 = I2Cx->SR1;
  flag2 = I2Cx->SR2;
  flag2 = flag2 << 16;

  /* Get the last event value from I2C status register */
  lastevent = (flag1 | flag2) & ((uint32_t)0x00FFFFFF);

  /* Check whether the last event contains the I2C_EVENT */
  if ((lastevent & I2C_EVENT) == I2C_EVENT)
  {
    /* SUCCESS: last event is equal to I2C_EVENT */
    status = SUCCESS;
  }
  else
  {
    /* ERROR: last event is different from I2C_EVENT */
    status = ERROR;
  }
  /* Return status */
  return status;
}

int CK_I2C_isI2CxInitialized(int n){
	if(n < 1 || n > 5){// Invalid
		return 2;
	}
	return isI2Cx_Initialized[n-1];
}


