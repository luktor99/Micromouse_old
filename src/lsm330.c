/*
 * lsm330.c
 *
 *  Created on: Feb 28, 2015
 *      Author: £ukasz
 */
#include "stm32f10x.h"
#include "lsm330.h"

void LSM330_Init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStruct;
	// SPI2 SCK, MISO, MOSI
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOB, &GPIO_InitStruct);

  	// CS_A - PA10
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStruct);

  	// CS_G - PC9
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStruct);

  	// CS_A, CS_G - ustaw stan wysoki
  	LSM330_CS_A_Disable();
  	LSM330_CS_G_Disable();

  	// DEN_G - stan wysoki
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_WriteBit(GPIOC, GPIO_Pin_15, 1);

	// Konfiguracja SPI2
	SPI_InitTypeDef SPI_InitStruct;
	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStruct);

	// Wlacz SPI2
	SPI_Cmd(SPI2, ENABLE);
}

void LSM330_Conf(uint8_t accsens, uint8_t gyrosens) {
	// Acc on, ODR: 100 Hz
	LSM330_AccWriteRegister(CTRL_REG1_A, 0b01010111);
	// zakres:
	LSM330_AccWriteRegister(CTRL_REG4_A, accsens);

	// Gyro on, ODR: 760Hz, Cut-off: 30
	LSM330_GyroWriteRegister(CTRL_REG1_G, 0b11001111);

	// HPF off
	//LSM330_GyroWriteRegister(CTRL_REG2_G, 0b00000000);

	// zakres:
	LSM330_GyroWriteRegister(CTRL_REG4_G, gyrosens);

	// HPF off
	//LSM330_GyroWriteRegister(CTRL_REG5_G, 0b00000000);
}

int16_t LSM330_GetAccX() {
	char byteH, byteL;
    LSM330_AccReadRegister(OUT_X_H_A, &byteH);
    LSM330_AccReadRegister(OUT_X_L_A, &byteL);
    return (int16_t)((byteH<<8)|byteL);
}
int16_t LSM330_GetAccY() {
	char byteH, byteL;
    LSM330_AccReadRegister(OUT_Y_H_A, &byteH);
    LSM330_AccReadRegister(OUT_Y_L_A, &byteL);
    return (int16_t)((byteH<<8)|byteL);
}
int16_t LSM330_GetAccZ() {
	char byteH, byteL;
    LSM330_AccReadRegister(OUT_Z_H_A, &byteH);
    LSM330_AccReadRegister(OUT_Z_L_A, &byteL);
    return (int16_t)((byteH<<8)|byteL);
}
int16_t LSM330_GetGyroX() {
	char byteH, byteL;
    LSM330_GyroReadRegister(OUT_X_H_G, &byteH);
    LSM330_GyroReadRegister(OUT_X_L_G, &byteL);
    return (int16_t)((byteH<<8)|byteL);
}
int16_t LSM330_GetGyroY() {
	char byteH, byteL;
    LSM330_GyroReadRegister(OUT_Y_H_G, &byteH);
    LSM330_GyroReadRegister(OUT_Y_L_G, &byteL);
    return (int16_t)((byteH<<8)|byteL);
}
int16_t LSM330_GetGyroZ() {
	char byteH, byteL;
    LSM330_GyroReadRegister(OUT_Z_H_G, &byteH);
    LSM330_GyroReadRegister(OUT_Z_L_G, &byteL);
    return (int16_t)((byteH<<8)|byteL);
}

void LSM330_CS_A_Enable(void) {
  	GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_RESET);
}
void LSM330_CS_G_Enable(void) {
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_RESET);
}
void LSM330_CS_A_Disable(void) {
  	GPIO_WriteBit(GPIOA, GPIO_Pin_10, Bit_SET);
}
void LSM330_CS_G_Disable(void) {
	GPIO_WriteBit(GPIOC, GPIO_Pin_9, Bit_SET);
}
char SPI_Transmit(char data) {
	SPI_I2S_SendData(SPI2, data);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
	return (char)SPI_I2S_ReceiveData(SPI2);
}
void LSM330_AccWriteRegister(char addr, char data) {
	LSM330_CS_A_Enable();
	SPI_Transmit(LSM330_WRITE|LSM330_NO_INC|addr);
	SPI_Transmit(data);
	LSM330_CS_A_Disable();
}
void LSM330_AccReadRegister(char addr, char *ret) {
	LSM330_CS_A_Enable();
	SPI_Transmit(LSM330_READ|LSM330_NO_INC|addr);
	*ret = (char)SPI_Transmit(0xFF);
	LSM330_CS_A_Disable();
}
void LSM330_GyroWriteRegister(char addr, char data) {
	LSM330_CS_G_Enable();
	SPI_Transmit(LSM330_WRITE|LSM330_NO_INC|addr);
	SPI_Transmit(data);
	LSM330_CS_G_Disable();
}
void LSM330_GyroReadRegister(char addr, char *ret) {
	LSM330_CS_G_Enable();
	SPI_Transmit(LSM330_READ|LSM330_NO_INC|addr);
	*ret = (char)SPI_Transmit(0xFF);
	LSM330_CS_G_Disable();
}
