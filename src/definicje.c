// inicjalizacje zmiennych
#include <stdint.h>
#include "stm32f10x.h"
#include "definicje.h"
#include "u8g_arm.h"

float CELL_VMAX=CELL_VMAX_default;

int32_t sum_start=0, sum_end=0, sum_thresh1=0, sum_thresh2=0;
uint8_t state=S_STOP;
uint32_t state_timer=0;

uint8_t moves[200];
uint16_t movescounter=0;
uint8_t movesstate=0;

float avgBL=0.0, avgBR=0.0;
float avgPL=0.0, avgPR=0.0;
float avgSL=0.0, avgSR=0.0;
uint16_t avgdistBR[N_AVG]={0};
uint16_t avgdistBL[N_AVG]={0};
uint16_t avgdistPR[N_AVG]={0};
uint16_t avgdistPL[N_AVG]={0};
uint16_t avgdistSR[N_AVG]={0};
uint16_t avgdistSL[N_AVG]={0};
uint16_t avg_counter=0;
float errorlastPLR=0.0;
float errorsumlastPLR=0.0;
float calibPLR=0.0;
float calibsumPLR=160.0;

float avgshortBL=0.0, avgshortBR=0.0;
float avgshortPL=0.0, avgshortPR=0.0;
float avgshortSL=0.0, avgshortSR=0.0;
uint16_t avgshortdistBR[N_AVGSHORT]={0};
uint16_t avgshortdistBL[N_AVGSHORT]={0};
uint16_t avgshortdistPR[N_AVGSHORT]={0};
uint16_t avgshortdistPL[N_AVGSHORT]={0};
uint16_t avgshortdistSR[N_AVGSHORT]={0};
uint16_t avgshortdistSL[N_AVGSHORT]={0};
uint16_t avgshort_counter=0;

//zmienne do obslugi gyro
float gyroint=0.0, gyroavg=-56.3922, gyrodest=0.0;
float gyroprev[3]={0.0};

//bufor do wysylania przez UART oraz bufory tekstow do ekranu
uint8_t counterUART=0;

// czasy swiecenia diod od czujnikow
uint16_t distances[6]={0};
uint16_t SPL_time=1500;
uint16_t SPR_time=1500;
uint16_t SBL_time=1500;
uint16_t SBR_time=1500;
uint16_t SSL_time=400;
uint16_t SSR_time=260;

// zmienne do obslugi enkoderow
int32_t EncodersR=1000000000;
int32_t EncodersL=1000000000;

float errorlast=0.0;
float iterm=0.0;

// zmienne pomocnicze do regulatorow
float errorlastwallrot=0.0;
float itermwallrot=0.0;
float errorlastwalltrans=0.0;
float itermwalltrans=0.0;
float wallPcalibrot=0.0;
float wallPcalibtrans=0.0;
float errorlastenc=0.0;
float itermenc=0.0;
float wallBcalib=0.0;

float wallBcalibL=0.0;
float wallBcalibR=0.0;

uint8_t walllast=W_LEFT;

float destLF=0.0, destRF=0.0, speedTrans=0.0, speedRot=0.0, itermL=0.0, itermR=0.0;
int32_t destL=0;
int32_t destR=0;
int32_t errorlastL=0;
int32_t errorlastR=0;

uint8_t sensors_state=0;
volatile uint8_t PID_enabled=0;

char buffer[50], bufferLiPo1[14], bufferLiPo2[7], bufferCharge[20];
char bufferAX[20], bufferAY[20], bufferAZ[20], bufferGX[20], bufferGY[20], bufferGZ[20];
char buffer1[10], buffer2[10], buffer3[10], buffer4[10], buffer5[10], buffer6[10];

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
	// 2 Przï¿½d Lewy		ADC3 -> ADC_Channel_14
	// 3 Przï¿½d Prawy	ADC4 -> ADC_Channel_11
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
    TIM_Cmd(TIM3, ENABLE);
    TIM3->CNT = 0;
    PID_enabled=0;
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
void PID_On(void) {
	itermL=0.0;
	itermR=0.0;
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);
    PID_enabled=1;

    gyrodest=gyroint;

    sensors_state=1;
}
void PID_Off(void) {
    //TIM_Cmd(TIM3, DISABLE);
    PID_enabled=0;
    setMotors(0,0);

	speedTrans=0.0;
	speedRot=0.0;

	state_timer=0;
	state=S_STOP;

	sensors_state=0;
}
