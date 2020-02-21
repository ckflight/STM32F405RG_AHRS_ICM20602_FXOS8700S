
#ifndef CK_I2C_H_
#define CK_I2C_H_

#include "stm32f4xx.h"

#define CK_RCC_APB1ENR_I2C1_ENABLE			1u<<21
#define CK_RCC_APB1ENR_I2C2_ENABLE			1u<<22
#define CK_RCC_APB1ENR_I2C3_ENABLE			1u<<23

#define CK_I2C_CR1_START						1u<<8
#define CK_I2C_CR1_STOP						1u<<9
#define CK_I2C_CR1_ACK						1u<<10
#define CK_I2C_CR1_PE						1u<<0

#define CK_I2C_CCR_FM						1u<<15
#define CK_I2C_CCR_DUTY						1u<<14

#define CK_I2C_SR1_ADDR						1u<<1
#define CK_I2C_SR1_SB						1u<<0
#define CK_I2C_SR1_TXE						1u<<7
#define CK_I2C_SR1_BTF						1u<<2

#define  I2C_EVENT_MASTER_BYTE_RECEIVED                    ((uint32_t)0x00030040)  /* BUSY, MSL and RXNE flags */
#define I2C_EVENT_MASTER_BYTE_TRANSMITTING                 ((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
#define  I2C_EVENT_MASTER_BYTE_TRANSMITTED                 ((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */
#define  I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED        ((uint32_t)0x00070082)  /* BUSY, MSL, ADDR, TXE and TRA flags */
#define  I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED           ((uint32_t)0x00030002)  /* BUSY, MSL and ADDR flags */

#define TIMEOUT	300000

typedef enum{

	CK_I2C_Transmit					=0,
	CK_I2C_Receive					=1

}CK_I2C_Mode;

typedef enum{

	CK_I2C_ACKDisable				=0,
	CK_I2C_ACKEnable					=1

}CK_I2C_ACK_Mode;

typedef enum{

	CK_I2C_100Khz					=0,
	CK_I2C_400Khz					=1

}CK_I2C_Speed;

void CK_I2C_Init(I2C_TypeDef* i2c,CK_I2C_Speed freq);

void CK_I2C_Transfer(uint8_t slaveAddress, uint8_t reg, uint8_t data);

void CK_I2C_ReadMulti(uint8_t slaveAddress, uint8_t reg, uint8_t* dataStore, int quantity);

uint8_t CK_I2C_ReadAck(I2C_TypeDef* I2Cx);

uint8_t CK_I2C_ReadNack(I2C_TypeDef* I2Cx);

void CK_I2C_START(uint8_t slaveAddress, CK_I2C_Mode mode, CK_I2C_ACK_Mode ack);

void CK_I2C_STOP(void);

ErrorStatus I2C_CheckEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT);

int CK_I2C_isI2CxInitialized(int n);


#endif /* CK_I2C_H_ */
