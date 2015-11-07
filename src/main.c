#include <stddef.h>
#include "stm32f10x.h"
#include "u8g_arm.h"
#include "lsm330.h"
#include "definicje.h"
#include "algorytm.h"


//------ DEBUG
float globaldebug;

uint8_t send=0;
uint8_t sensors_state=0;

//------

int main(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// inicjalizacja UARTa (Bluetooth)
	UART_Init();
	// inicjalizacja wyswietlacza
	//OLED_Init();
	// inicjalizacja czujnikow zblizeniowych i ogolnie ADC
	SENSORS_Init();
	// przerwania od enkoderow
	Encoders_Init();
	// mostek H
	HBridge_Init();
	// inicjalizacja LSM330 i ustawienie czulosci
	LSM330_Init();
	LSM330_Conf(ACC_2g, GYRO_2000dps);
	// czas do PID
	PID_Init();

	// joystick...
	/*GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStruct);*/

	reportbattery();

	sprintf(buffer, "Restarted\n\r");
	UART_Send();


	// -------------
	uint32_t mi = 0, mj = 0;

	//calibrate_gyro();

	while (1)
	{

		/*sprintf(buffer1, "Ax %d", LSM330_GetAccX());
		sprintf(buffer2, "Ay %d", LSM330_GetAccY());
		sprintf(buffer3, "Az %d", LSM330_GetAccZ());
		sprintf(buffer4, "Gx %d", LSM330_GetGyroX());
		sprintf(buffer5, "Gy %d", LSM330_GetGyroY());
		sprintf(buffer6, "Gz %d", LSM330_GetGyroZ());
		// odswiezenie wyswietlacza
		OLED_buff_LiPo();
		OLED_Refresh();*/


		// odczytaj czujniki
		if(sensors_state) {
			readSP();
			//readSS();
			readSB();
			avgcalc();
		}
        if(send) {
        	//sprintf(buffer, "err*1000: %d\n\rgyroint: %d\n\r", (int32_t)(errorlastwallrot*1000.0), (int32_t)gyroint);
        	sprintf(buffer, "BL: %d\n\rBR: %d\n\r", (int32_t)((avgBL)*1000.0), (int32_t)((avgBR)*1000.0));
        	UART_Send();
			send=0;
        }

		mi=3000;
		while(mi--);

		mj++;
	}
}
void UART_Init(void) {

	// PC10 - USART3_TX, PC11 - USART3_RX
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Remapowanie USART3 na piny PC10 i PC11
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);

	USART_InitTypeDef USART_InitStruct;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;

	USART_DeInit(USART3);
	USART_Init(USART3, &USART_InitStruct);
	// wlaczamy USART3
	USART_Cmd(USART3, ENABLE);

    // wlaczamy przerwanie dla danych przychodzacych
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART3_IRQn);
}
void UART_Send(void) {
	uint16_t n=0;
	while(buffer[n]!='\0') {
		USART_SendData(USART3, buffer[n]);
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		n++;
	}
}
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        char data=(char)USART_ReceiveData(USART3);
        /*bufferUART[counterUART++] = data;
        if(counterUART>=4) {
        	TIM_SetCompare1(TIM1, (bufferUART[0]<<8)|(bufferUART[1]));
        	TIM_SetCompare2(TIM1, (bufferUART[2]<<8)|(bufferUART[3]));
        	distances[PL]=(bufferUART[0]<<8)|(bufferUART[1]);
        	distances[PR]=(bufferUART[2]<<8)|(bufferUART[3]);
        	counterUART=0;
        }*/

        if(data=='g') {sensors_state=1; calibrate_gyro(); sprintf(buffer, "GyroAvg: %d.%d\n\r", (int)gyroavg, (gyroavg-(int)(gyroavg))*100); UART_Send();}
        if(data=='9') {PID_On(); movescounter=0; movesstate=1;}
        if(data=='0') {PID_Off(); movescounter=0; movesstate=0;}
        if(data=='b') reportbattery();
        if(data=='r') send=1;

        if(data=='h') {
        	gyrodest=gyroint;
        	uint16_t cntr;
        	float temp1=0.0, temp2=0.0, temp3=0.0, temp4=0.0, temp5=0.0;
        	for(cntr=0; cntr<50000; cntr++) {
        		temp1+=(avgPR-avgPL)/(avgPR+avgPL+0.0000001);
        		temp2+=avgPR+avgPL;
        		temp3+=(avgSR-avgSL)/(avgSR+avgSL+0.0000001);
        		temp4+=avgBL;
        		temp5+=avgBR;
        	}
        	wallPcalibrot=temp1/50000.0;
        	wallPcalibtrans=temp2/50000.0;
        	wallBcalib=temp3/50000.0;
        	wallBcalibL=temp4/50000.0;
        	wallBcalibR=temp5/50000.0;

        	moves[0]=M_L;
        	moves[1]=M_L;
        	moves[2]=M_SCAN;

        	mazeinit();
        	USART_SendData(USART3, 'K');
        }

        if(state==S_STOP) {
			if(data=='a') {gyrodest+=GYRO_90;}
			if(data=='d') {gyrodest-=GYRO_90;}
			if(data=='1' || data=='w') {forward(1);}
			if(data=='2') {forward(2);}
			if(data=='3') {forward(3);}
			if(data=='4') {forward(4);}
			if(data=='5') {forward(5);}
			if(data=='6') {forward(6);}
			if(data=='7') {forward(7);}
			if(data=='8') {forward(8);}
        }

        if(data=='m') {
        	int x;
        	sprintf(buffer, "\n\rMAPA:\n\r|");
        	UART_Send();
        	for(x=15; x>=0; x--) {
        		int y;
        		for(y=0; y<16; y++) {
        				sprintf(buffer, " %d |", mapa[x][y]);
        				UART_Send();
				}
				sprintf(buffer, "\n\r|");
				UART_Send();
			}
        }

        if(data=='.') {

        	sensors_state=1;

    		sensor_front=(wallPcalibtrans-(avgshortPR+avgshortPL));
    		sensor_left=avgshortBL;
    		sensor_right=avgshortBR;


    		sprintf(buffer, "sf: %u, sl: %u, sr: %u, kier: %d\n\r", sensor_front, sensor_left, sensor_right, kierunek);
    		UART_Send();
        }

        if(data=='w') {movesstate=1;}
        if(data=='z') {
        	moves[0]=M_L;
        	moves[1]=M_L;
        	moves[2]=M_SCAN;

        	maze_reinit();
        	USART_SendData(USART3, 'Z');
        }


        //if(data=='=') {Kp+=0.001; send=1;}
        //if(data=='-') {Kp-=0.001; send=1;}

		//USART_SendData(USART3, data);
		//while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
}
void SENSORS_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1; // liczba kanalow
	ADC_Init(ADC1, &ADC_InitStructure);

	// definiujemy kolejnosc wybranych kanalow
	// 0 Bok Lewy		ADC6 -> ADC_Channel_15
	// 1 Bok Prawy		ADC1 -> ADC_Channel_10
	// 2 PrzĂŻÂżÂ˝d Lewy		ADC3 -> ADC_Channel_14
	// 3 PrzĂŻÂżÂ˝d Prawy	ADC4 -> ADC_Channel_11
	// 4 Skos Lewy		ADC5 -> ADC_Channel_13
	// 5 Skos Prawy		ADC1 -> ADC_Channel_12
	/*
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 3, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 5, ADC_SampleTime_71Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_71Cycles5);
	*/

	ADC_Cmd(ADC1, ENABLE);

	// kalibracja ADC
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	//wyjscia na diody:
	/*
	 * PA4  -> BR
	 * PA5  -> PR
	 * PA6  -> SR
	 * PA7  -> SL
	 * PB10 -> BL
	 * PB11 -> PL
	 */

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}
void OLED_Init(void) {
	// delay potrzebny zeby wyswietlacz wystartowal
	uint32_t delay=300000;
	while(delay--);
	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
	u8g_SetDefaultForegroundColor(&u8g);
}
void OLED_Draw(void) {
	u8g_SetDefaultForegroundColor(&u8g);
	u8g_SetFont(&u8g, u8g_font_profont15);

	u8g_DrawStr(&u8g, 0, 17, buffer1);
	u8g_DrawStr(&u8g, 64, 17, buffer2);
	u8g_DrawStr(&u8g, 0, 28, buffer3);
	u8g_DrawStr(&u8g, 64, 28, buffer4);
	u8g_DrawStr(&u8g, 0, 39, buffer5);
	u8g_DrawStr(&u8g, 64, 39, buffer6);


	// PASEK
	u8g_SetFont(&u8g, u8g_font_baby);
	//u8g_SetColorIndex(&u8g, 1);
	u8g_DrawBox(&u8g, 0, 0, 127, 7);
	u8g_SetColorIndex(&u8g, 0);
	//u8g_DrawPixel(28, 14);
	u8g_DrawStr(&u8g, 1, 6, "uHulk");
	u8g_DrawStr(&u8g, 30, 6, bufferLiPo1);
	u8g_DrawStr(&u8g, 60, 6, bufferLiPo2);
	u8g_DrawStr(&u8g, 110, 6, bufferCharge);
}
inline void OLED_Refresh(void) {
	u8g_FirstPage(&u8g);
	do
	{
		OLED_Draw();
	} while ( u8g_NextPage(&u8g) );
}
void OLED_buff_LiPo(void) {
	uint16_t ADC_LiPo[2];

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_71Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	ADC_LiPo[0] = ADC_GetConversionValue(ADC1);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_71Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	ADC_LiPo[1] = ADC_GetConversionValue(ADC1);

	float LiPo0 = (ADC_LiPo[0]/4095.0)*9.102684563758389;
	float LiPo2 = (ADC_LiPo[1]/4095.0)*4.325961538461538;
	float LiPo1 = LiPo0 - LiPo2;

	//y=96.274*x*x-600.16*x+932.94
	uint8_t LiPoCharge=(uint8_t)(24.0685*LiPo0*LiPo0-300.08*LiPo0+932.94);

	uint8_t LiPo1H = (uint8_t)LiPo1;
	uint8_t LiPo1L = ((uint16_t)(LiPo1*100.0))%100;
	uint8_t LiPo2H = (uint8_t)LiPo2;
	uint8_t LiPo2L = ((uint16_t)(LiPo2*100.0))%100;
	char c1=(LiPo1L<10)?'0':24;
	char c2=(LiPo2L<10)?'0':24;

	sprintf(bufferLiPo1, "(%u.%c%uV,", LiPo1H, c1, LiPo1L);
	sprintf(bufferLiPo2, "%u.%c%uV)", LiPo2H, c2, LiPo2L);
	sprintf(bufferCharge, "%u%%", LiPoCharge);
}
uint16_t readSensor(uint8_t channel) {
	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_71Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
void HBridge_Init(void) {
	//PWMA - PA9 - OC2, PWMB - PA8 - OC1
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	//AIN1 - PC12, AIN2 - PD2, BIN1 - PB4, BIN2 - PB5
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // Disable JTAG/SWD so pins are available
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_InitStruct.TIM_Prescaler = 0;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_Period = 30000;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_InitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_InitStruct);

    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct); // to zapobiega problemowi w Eclipse
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_Pulse = 0;
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM1, &TIM_OCInitStruct);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStruct);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

	GPIO_WriteBit(GPIOC, GPIO_Pin_12, 0);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, 0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_4, 0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_5, 0);
}
void readSB(void) {
	uint32_t del;

	distances[BL] = readSensor(S_BL);
	BL_ON;
	del=SBL_time;
	while(del--);
	distances[BL] -= readSensor(S_BL);
	BL_OFF;

	distances[BR] = readSensor(S_BR);
	BR_ON;
	del=SBR_time;
	while(del--);
	distances[BR] -= readSensor(S_BR);
	BR_OFF;

	if(distances[BL]>3000) distances[BL]=0;
	if(distances[BR]>3000) distances[BR]=0;
}
void readSP(void) {
	uint32_t del;

	distances[PL] = readSensor(S_PL);
	PL_ON;
	del=SPL_time;
	while(del--);
	distances[PL] -= readSensor(S_PL);
	PL_OFF;

	distances[PR] = readSensor(S_PR);
	PR_ON;
	del=SPR_time;
	while(del--);
	distances[PR] -= readSensor(S_PR);
	PR_OFF;

	if(distances[PL]>3000) distances[PL]=0;
	if(distances[PR]>3000) distances[PR]=0;
}
void readSS(void) {
	uint32_t del;

	distances[SL] = readSensor(S_SL);
	SL_ON;
	del=SSL_time;
	while(del--);
	distances[SL] -= readSensor(S_SL);
	SL_OFF;

	distances[SR] = readSensor(S_SR);
	SR_ON;
	del=SSR_time;
	while(del--);
	distances[SR] -= readSensor(S_SR);
	SR_OFF;

	if(distances[SL]>3000) distances[SL]=0;
	if(distances[SR]>3000) distances[SR]=0;
}
void Encoders_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	NVIC_InitStruct.NVIC_IRQChannel = EXTI2_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

	EXTI_InitTypeDef EXTI_InitStruct;
	EXTI_InitStruct.EXTI_Line = EXTI_Line0;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStruct);

	EXTI_InitStruct.EXTI_Line = EXTI_Line2;
	EXTI_Init(&EXTI_InitStruct);
}
void PID_Init(void) {
	TIM_DeInit(TIM3);
	TIM_InternalClockConfig(TIM3);

    TIM_TimeBaseInitTypeDef TIM_InitStruct;
    TIM_InitStruct.TIM_Prescaler = 7200-1;
    TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStruct.TIM_Period = PID_interval-1;
    TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_InitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_InitStruct);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
    TIM3->CNT = 0;

    NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

    //TIM_Cmd(TIM3, ENABLE);
    TIM3->CNT = 0;
}
void setMotors(int32_t pR, int32_t pL) {
	uint32_t absR=abs(pR);
	uint32_t absL=abs(pL);

	TIM_SetCompare1(TIM1, (absR>pwm_max)?pwm_max:absR);
	GPIO_WriteBit(GPIOB, GPIO_Pin_4, pR>0);
	GPIO_WriteBit(GPIOB, GPIO_Pin_5, pR<0);

	TIM_SetCompare2(TIM1, (absL>pwm_max)?pwm_max:absL);
	GPIO_WriteBit(GPIOC, GPIO_Pin_12, pL<0);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, pL>0);
}
void clamp_iterm(float *term, float max) {
	if(*term>max) *term=max;
	else if(*term<-max) *term=-max;
}
void reportbattery(void) {
	uint16_t ADC_LiPo[2];

	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 1, ADC_SampleTime_71Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	ADC_LiPo[0] = ADC_GetConversionValue(ADC1);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_71Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	ADC_LiPo[1] = ADC_GetConversionValue(ADC1);

	float LiPo0 = (ADC_LiPo[0]/4095.0)*9.102684563758389;
	float LiPo2 = (ADC_LiPo[1]/4095.0)*4.325961538461538;
	float LiPo1 = LiPo0 - LiPo2;

	uint8_t LiPo1H = (uint8_t)LiPo1;
	uint8_t LiPo1L = ((uint16_t)(LiPo1*100.0))%100;
	uint8_t LiPo2H = (uint8_t)LiPo2;
	uint8_t LiPo2L = ((uint16_t)(LiPo2*100.0))%100;
	char c1=(LiPo1L<10)?'0':24;
	char c2=(LiPo2L<10)?'0':24;

	sprintf(buffer, "LiPo: %u.%c%uV,%u.%c%uV\n\r", LiPo1H, c1, LiPo1L, LiPo2H, c2, LiPo2L);
	UART_Send();
}

void PID_On(void) {
	itermL=0.0;
	itermR=0.0;
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);

    gyrodest=gyroint;

    sensors_state=1;
}
void PID_Off(void) {
    TIM_Cmd(TIM3, DISABLE);
    setMotors(0,0);

	speedTrans=0.0;
	speedRot=0.0;

	state_timer=0;
	state=S_STOP;

	sensors_state=0;
}
void TIM3_IRQHandler(void) {
	//wylaczenie diod jakby co:
	char statePL = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11);
	char statePR = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5);
	char stateBL = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10);
	char stateBR = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4);
	PL_OFF;
	PR_OFF;
	BL_OFF;
	BR_OFF;


	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		// calkowanie gyro
		float reading=(float)LSM330_GetGyroZ()-gyroavg;
		gyroint+=(reading + 2.*gyroprev[0] + 2.*gyroprev[1] + gyroprev[2])/6.;
		gyroprev[2]=gyroprev[1];
		gyroprev[1]=gyroprev[0];
		gyroprev[0]=reading;


		//Profiler();
		float wallfix=0.0;
		// dorownywanie do sciany
		/*if(avgSL>WALL_PIDTHRESH && avgSR>WALL_PIDTHRESH && avgBL>30.0 && avgBR>30.0) {
			wallfix=10000.0*(wallBcalib-(avgSR-avgSL)/(avgSR+avgSL+0.0000001));
		} else */

		#define K_WALL 100.0
		if(walllast==W_LEFT) {
			if(avgBL>30.0) {
				wallfix=K_WALL*(wallBcalibL-avgBL);
				walllast=W_LEFT;
			} else if(avgBR>30.0) {
				wallfix=-K_WALL*(wallBcalibR-avgBR);
				walllast=W_RIGHT;
			}
		} else {
			if(avgBR>30.0) {
				wallfix=-K_WALL*(wallBcalibR-avgBR);
				walllast=W_RIGHT;
			} else if(avgBL>30.0) {
				wallfix=K_WALL*(wallBcalibL-avgBL);
				walllast=W_LEFT;
			}
		}


		if(state==S_FORWARD) {
			int32_t sum=EncodersL+EncodersR;
			if(sum<sum_thresh1) {
				speedTrans=PWM_ZERO+(CELL_VMAX)*((sum-sum_start)/sum_thresh1);
			} else
			if(sum<sum_thresh2) {
				speedTrans=PWM_ZERO+CELL_VMAX;
			} else {
				//czy jest sciana?
				if((wallPcalibtrans-(avgPR+avgPL))<33.0) { //95.0
					state=S_BRAKEWALL;
					wallfix=0.0;
				} else {
					state=S_BRAKEENC;
				}
			}
		} else if(state==S_ROTATION) {
			state_timer++;
			if(state_timer>100) { //0.4s
				state=S_STOP;
				state_timer=0;
			}
		} else if(state==S_BRAKEWALL) {
			float errorwalltrans=wallPcalibtrans-(avgPR+avgPL);
			float dtermtrans=errorwalltrans-errorlastwalltrans;
			itermwalltrans+=errorwalltrans;
			clamp_iterm(&itermwalltrans, 100);
			speedTrans=(120.0 * errorwalltrans + 1.0*itermwalltrans - 6.0*(dtermtrans));
			errorlastwalltrans=errorwalltrans;

			state_timer++;

			if(state_timer>50 && fabs(errorwalltrans)<1.0 && fabs(dtermtrans)<1.0) {
				state=S_WALLALIGN;
				state_timer=0;
			}
		}  else if(state==S_BRAKEENC) {
			int32_t sum=EncodersL+EncodersR;
			float errorenc=sum_end-sum;
			float dtermenc=errorenc-errorlastenc;
			itermenc+=errorenc;
			clamp_iterm(&itermenc, 30);
			//speedTrans=(120.0 * errorenc + 20.0*itermenc + 2.0*(dtermenc)); dziaĂŻÂżÂ˝a, ale polizg
			speedTrans=(80.0 * errorenc + 12.0*itermenc + 2.0*(dtermenc));
			errorlastenc=errorenc;

			float brakelimit=PWM_ZERO+(1-sum/sum_end)*CELL_VMAX;
			if(speedTrans>brakelimit) speedTrans=brakelimit;
			else if(speedTrans<-brakelimit) speedTrans=-brakelimit;

			state_timer++;
			if(state_timer>100 && abs(errorenc)==0 && abs(dtermenc)==0) {
				state=S_STOP;
				state_timer=0;
			}
		} else if(state==S_WALLALIGN) {
			float errorwallrot=(avgPR-avgPL)/(avgPR+avgPL+0.000001)-wallPcalibrot;
			gyroint+=7000.0*errorwallrot;

			state_timer++;
			if(state_timer>50 && abs(errorwallrot)<=0.0001) {
				state=S_STOP;
				state_timer=0;
			}
		} else if(state==S_STOP) {
			speedTrans=0.0;
			wallfix=0.0;

			if(movesstate==1) {
				if(moves[movescounter]==M_STOP) {
					movescounter=0;
					movesstate=0;
				} else if(moves[movescounter]==M_F1 || moves[movescounter]==M_F2 || moves[movescounter]==M_F3 || moves[movescounter]==M_F4 || moves[movescounter]==M_F5 || moves[movescounter]==M_F6 || moves[movescounter]==M_F7 || moves[movescounter]==M_F8 || moves[movescounter]==M_F9 || moves[movescounter]==M_F10 || moves[movescounter]==M_F11 || moves[movescounter]==M_F12 || moves[movescounter]==M_F13 || moves[movescounter]==M_F14 || moves[movescounter]==M_F15 || moves[movescounter]==M_F16) {
					CELL_VMAX=11000;//13500
					switch(moves[movescounter]) {
						case M_F1: CELL_VMAX=5000; forward(1); break;//5000
						case M_F2: CELL_VMAX=7000; forward(2); break;//8000
						case M_F3: CELL_VMAX=9000; forward(3); break;//12000
						case M_F4: forward(4); break;
						case M_F5: forward(5); break;
						case M_F6: forward(6); break;
						case M_F7: forward(7); break;
						case M_F8: forward(8); break;
						case M_F9: forward(9); break;
						case M_F10: forward(10); break;
						case M_F11: forward(11); break;
						case M_F12: forward(12); break;
						case M_F13: forward(13); break;
						case M_F14: forward(14); break;
						case M_F15: forward(15); break;
						case M_F16: forward(16); break;
					}
					movescounter++;
				} else if(moves[movescounter]==M_L) {
					gyrodest+=GYRO_90;
					state=S_ROTATION;
					movescounter++;
				} else if(moves[movescounter]==M_LL) {
					gyrodest+=2.0*GYRO_90;
					state=S_ROTATION;
					movescounter++;
				} else if(moves[movescounter]==M_R) {
					gyrodest-=GYRO_90;
					state=S_ROTATION;
					movescounter++;
				} else if(moves[movescounter]==M_SCAN) {
					//moves[0]=M_SCAN;
					movesstate=nextscanstep(); // 0 albo 1 w zaleznosci czy kontynuujemy
					movescounter=0;



					if(movesstate) USART_SendData(USART3, 's');
					else {
						sprintf(buffer, "\n\rKoniec!\n\r");
						UART_Send();
					}
				}
			}
		}

		float error=gyrodest-gyroint;
		iterm+=error;
		clamp_iterm(&iterm, MAX_I);
		speedRot=(int32_t)(Kp * error + Ki*itermL - Kd * (gyroint - errorlast));
		errorlast=gyroint;

		setMotors(speedTrans+speedRot+wallfix, speedTrans-speedRot-wallfix);
	}


	//przywrĂ�Âłcenie stanu diod:
	GPIO_WriteBit(GPIOB, GPIO_Pin_11, statePL);
	GPIO_WriteBit(GPIOA, GPIO_Pin_5, statePR);
	GPIO_WriteBit(GPIOB, GPIO_Pin_10, stateBL);
	GPIO_WriteBit(GPIOA, GPIO_Pin_4, stateBR);
}
void avgcalc(void) {
	// liczenie sredniej ruchomej
	avgdistBL[avg_counter]=distances[BL];
	avgdistBR[avg_counter]=distances[BR];
	avgdistPL[avg_counter]=distances[PL];
	avgdistPR[avg_counter]=distances[PR];
	avgdistSL[avg_counter]=distances[SL];
	avgdistSR[avg_counter]=distances[SR];

	avgshortdistBL[avgshort_counter]=distances[BL];
	avgshortdistBR[avgshort_counter]=distances[BR];
	avgshortdistPL[avgshort_counter]=distances[PL];
	avgshortdistPR[avgshort_counter]=distances[PR];
	avgshortdistSL[avgshort_counter]=distances[SL];
	avgshortdistSR[avgshort_counter]=distances[SR];

	uint32_t aBL=0, aBR=0, aPL=0, aPR=0, aSL=0, aSR=0;
	uint16_t i;
	for(i=0; i<N_AVG; i++) {
		aBL+=avgdistBL[i];
		aBR+=avgdistBR[i];
		aPL+=avgdistPL[i];
		aPR+=avgdistPR[i];
		aSL+=avgdistSL[i];
		aSR+=avgdistSR[i];
	}
	avgBL=(float)aBL/(float)N_AVG;
	avgBR=(float)aBR/(float)N_AVG;
	avgPL=(float)aPL/(float)N_AVG;
	avgPR=(float)aPR/(float)N_AVG;
	avgSL=(float)aSL/(float)N_AVG;
	avgSR=(float)aSR/(float)N_AVG;

	avg_counter=(avg_counter+1)%N_AVG;

	uint32_t ashortBL=0, ashortBR=0, ashortPL=0, ashortPR=0, ashortSL=0, ashortSR=0;

	for(i=0; i<N_AVGSHORT; i++) {
		ashortBL+=avgshortdistBL[i];
		ashortBR+=avgshortdistBR[i];
		ashortPL+=avgshortdistPL[i];
		ashortPR+=avgshortdistPR[i];
		ashortSL+=avgshortdistSL[i];
		ashortSR+=avgshortdistSR[i];
	}
	avgshortBL=(float)ashortBL/(float)N_AVGSHORT;
	avgshortBR=(float)ashortBR/(float)N_AVGSHORT;
	avgshortPL=(float)ashortPL/(float)N_AVGSHORT;
	avgshortPR=(float)ashortPR/(float)N_AVGSHORT;
	avgshortSL=(float)ashortSL/(float)N_AVGSHORT;
	avgshortSR=(float)ashortSR/(float)N_AVGSHORT;

	avgshort_counter=(avgshort_counter+1)%N_AVGSHORT;
}
void calibrate_gyro(void) {
	//kalibracja gyro:
	uint32_t i;
	for(i=0; i<20000; i++) {
		gyroint+=(float)LSM330_GetGyroZ();
	}
	gyroavg=gyroint/20000.;
	gyroint=0.0;
}

void EXTI0_IRQHandler(void) {
	EncodersR+=-1+2*GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1);

	EXTI_ClearITPendingBit(EXTI_Line0);
}
void EXTI2_IRQHandler(void) {
	EncodersL+=1-2*GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3);

	EXTI_ClearITPendingBit(EXTI_Line2);
}
void forward(uint8_t n) {
	sum_start=EncodersL+EncodersR;
	sum_end=(int32_t)(CELL_SIZE*2.0*n)+sum_start;
	sum_thresh1=sum_start+CELL_STOP;
	sum_thresh2=sum_end-CELL_STOP;

	state=S_FORWARD;
}

/*
 * float errorwalltrans=wallPcalibtrans-(avgPR+avgPL);
			float dtermtrans=errorwalltrans-errorlastwalltrans;
			itermwalltrans+=errorwalltrans;
			clamp_iterm(&itermwalltrans, 100);
			speedTrans=(120.0 * errorwalltrans + 1.0*itermwalltrans - 6.0*(dtermtrans));
			errorlastwalltrans=errorwalltrans;

			float errorwallrot=(avgPR-avgPL)/(avgPR+avgPL+0.000001)-wallPcalibrot;
			float dtermrot=errorwallrot-errorlastwallrot;
			itermwallrot+=errorwallrot;
			clamp_iterm(&itermwallrot, 10);
			gyroint+=(10000.0 * errorwallrot + 8.0*itermwallrot - 160.0*(dtermrot));
			errorlastwallrot=errorwallrot;

			state_timer++;

			if(state_timer>50 && fabs(errorwalltrans)<=1.0 && fabs(dtermtrans)<=0.8 && fabs(errorwallrot)<=0.005 && fabs(dtermrot)<=0.005) {
				state=S_STOP;
				state_timer=0;
			}
 *
 */
