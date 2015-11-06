/*
 * lsm330.h
 *
 *  Created on: Feb 28, 2015
 *      Author: £ukasz
 */

#ifndef LSM330_H_
#define LSM330_H_

// lista rejestrow
#define CTRL_REG1_A 0x20
#define CTRL_REG2_A 0x21
#define CTRL_REG3_A 0x22
#define CTRL_REG4_A 0x23
#define CTRL_REG5_A 0x24
#define CTRL_REG6_A 0x25
#define REFERENCE_A 0x26
#define STATUS_REG_A 0x27
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D
#define FIFO_CTRL_REG 0x2E
#define FIFO_SRC_REG 0x2F
#define INT1_CFG_A 0x30
#define INT1_SOURCE_A 0x31
#define INT1_THS_A 0x32
#define INT1_DURATION_A 0x33
#define INT2_CFG_A 0x34
#define INT2_SOURCE_A 0x35
#define INT2_THS_A 0x36
#define INT2_DURATION_A 0x37
#define CLICK_CFG_A 0x38
#define CLICK_SRC_A 0x39
#define CLICK_THS_A 0x3A
#define TIME_LIMIT_A 0x3B
#define TIME_LATENCY_A 0x3C
#define TIME_WINDOW_A 0x3D
#define Act_THS 0x3E
#define Act_DUR 0x3F
#define WHO_AM_I_G 0x0F
#define CTRL_REG1_G 0x20
#define CTRL_REG2_G 0x21
#define CTRL_REG3_G 0x22
#define CTRL_REG4_G 0x23
#define CTRL_REG5_G 0x24
#define REFERENCE_G 0x25
#define OUT_TEMP_G 0x26
#define STATUS_REG_G 0x27
#define OUT_X_L_G 0x28
#define OUT_X_H_G 0x29
#define OUT_Y_L_G 0x2A
#define OUT_Y_H_G 0x2B
#define OUT_Z_L_G 0x2C
#define OUT_Z_H_G 0x2D
#define FIFO_CTRL_REG_G 0x2E
#define FIFO_SRC_REG_G 0x2F
#define INT1_CFG_G 0x30
#define INT1_SRC_G 0x31
#define INT1_TSH_XH_G 0x32
#define INT1_TSH_XL_G 0x33
#define INT1_TSH_YH_G 0x34
#define INT1_TSH_YL_G 0x35
#define INT1_TSH_ZH_G 0x36
#define INT1_TSH_ZL_G 0x37
#define INT1_DURATION_G 0x38

#define LSM330_READ		0b10000000
#define LSM330_WRITE	0b00000000
#define LSM330_INC		0b00000000
#define LSM330_NO_INC	0b00000000

#define ACC_2g 0b00000000
#define ACC_4g 0b00010000
#define ACC_8g 0b00100000
#define ACC_16g 0b00110000
#define ACC_HIGHRES 0b00001000

#define GYRO_250dps 0b00000000
#define GYRO_500dps 0b00010000
#define GYRO_2000dps 0b00100000




void LSM330_Init(void);
void LSM330_AccReadRegister(char addr, char *ret);
void LSM330_AccWriteRegister(char addr, char data);
void LSM330_GyroReadRegister(char addr, char *ret);
void LSM330_GyroWriteRegister(char addr, char data);
void LSM330_CS_A_Enable(void);
void LSM330_CS_G_Enable(void);
void LSM330_CS_A_Disable(void);
void LSM330_CS_G_Disable(void);
void LSM330_Conf(uint8_t, uint8_t);


#endif /* LSM330_H_ */
