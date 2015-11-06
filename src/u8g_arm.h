#ifndef _U8G_ARM_H
#define _U8G_ARM_H

#include "u8g.h"

#define SYS_CLK 72000000
#define DELAY_TIM_FREQUENCY 1000000 /* = 1MHZ -> timer runs in microseconds */
#define SSD1306_I2C_ADDRESS   0x78	// 011110+SA0+RW - 0x3C or 0x3D

#define SSD1306_SEGREMAP 0xA0
#define SSD1306_COMSCANINC 0xC0

u8g_t u8g;

void delay_init();
void delay_system_ticks(uint32_t sys_ticks);
void delay_micro_seconds(uint32_t us);
void I2C_stop();
void I2C_start( uint8_t address, uint8_t direction);

void i2c_init(uint32_t ns) U8G_NOINLINE;
void i2c_out(uint8_t data);
void i2c_command(uint8_t data);
uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr);

#endif
