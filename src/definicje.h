/*
 * czujniki.h
 *
 *  Created on: Mar 5, 2015
 *      Author: £ukasz
 */

#ifndef DEFINICJE_H_
#define DEFINICJE_H_

// max wartosc PWM
#define pwm_max 30000

// ustawienia labiryntu i predkosci
#define CELL_SIZE 201.781
#define PWM_ZERO 5000
#define CELL_VMAX_default 13500
extern float CELL_VMAX;
#define CELL_STOP 170 //150!
#define GYRO_90 662000
#define WALL_PIDTHRESH 20.0
#define BRAKE_LIMIT PWM_ZERO+CELL_VMAX

// stany
extern int32_t sum_start, sum_end, sum_thresh1, sum_thresh2;
enum state123 {S_FORWARD, S_BRAKEWALL, S_BRAKEENC, S_WALLALIGN, S_ROTATION, S_STOP};
extern uint8_t state;
extern uint32_t state_timer;

// sekwencje
enum allmoves {M_F1, M_F2, M_F3, M_F4, M_F5, M_F6, M_F7, M_F8, M_F9, M_F10, M_F11, M_F12, M_F13, M_F14, M_F15, M_F16, M_L, M_R, M_LL, M_STOP, M_SCAN};
//uint8_t moves[] = {M_L, M_L, M_F1, M_R, M_F1, M_L, M_F3, M_R, M_F3, M_L, M_F2, M_L, M_F1, M_L, M_L, M_F1, M_R, M_F2, M_R, M_F4, M_R, M_F1, M_R, M_F1, M_L, M_F1, M_R, M_F1, M_L, M_F1, M_R, M_F1, M_R, M_F1, M_L, M_F1, M_L, M_F4, M_R, M_F6, M_R, M_F4, M_R, M_F2, M_R, M_F2, M_L, M_F1, M_L, M_F1, M_L, M_F1, M_R, M_F1, M_L, M_F1, M_L, M_F2, M_R, M_F1, M_L, M_F2, M_L, M_F6, M_L, M_F2, M_R, M_F1, M_R, M_F2, M_L, M_F3, M_L, M_F10, M_STOP};
extern uint8_t moves[200];
extern uint16_t movescounter;
extern uint8_t movesstate;

// srednie ruchome do filtrowania czunikow
#define N_AVG 18
extern float avgBL, avgBR;
extern float avgPL, avgPR;
extern float avgSL, avgSR;
extern uint16_t avgdistBR[N_AVG];
extern uint16_t avgdistBL[N_AVG];
extern uint16_t avgdistPR[N_AVG];
extern uint16_t avgdistPL[N_AVG];
extern uint16_t avgdistSR[N_AVG];
extern uint16_t avgdistSL[N_AVG];
extern uint16_t avg_counter;
extern float errorlastPLR;
extern float errorsumlastPLR;
extern float calibPLR;
extern float calibsumPLR;

#define N_AVGSHORT 3
extern float avgshortBL, avgshortBR;
extern float avgshortPL, avgshortPR;
extern float avgshortSL, avgshortSR;
extern uint16_t avgshortdistBR[N_AVGSHORT];
extern uint16_t avgshortdistBL[N_AVGSHORT];
extern uint16_t avgshortdistPR[N_AVGSHORT];
extern uint16_t avgshortdistPL[N_AVGSHORT];
extern uint16_t avgshortdistSR[N_AVGSHORT];
extern uint16_t avgshortdistSL[N_AVGSHORT];
extern uint16_t avgshort_counter;

//zmienne do obslugi gyro
extern float gyroint, gyroavg, gyrodest;
extern float gyroprev[3];

//bufor do wysylania przez UART oraz bufory tekstow do ekranu
extern char bufferUART[6];
extern uint8_t counterUART;
extern char buffer[50], bufferLiPo1[14], bufferLiPo2[7], bufferCharge[20];
extern char bufferAX[20], bufferAY[20], bufferAZ[20], bufferGX[20], bufferGY[20], bufferGZ[20];
extern char buffer1[10], buffer2[10], buffer3[10], buffer4[10], buffer5[10], buffer6[10];

// czasy swiecenia diod od czujnikow
extern uint16_t distances[6];
extern uint16_t SPL_time;
extern uint16_t SPR_time;
extern uint16_t SBL_time;
extern uint16_t SBR_time;
extern uint16_t SSL_time;
extern uint16_t SSR_time;

// zmienne do obslugi enkoderow
extern int32_t EncodersR;
extern int32_t EncodersL;

// nastawy PID
#define PID_interval 20 //10 -> 1000Hz 20->500Hz

//PID Ku=0.085 Tu=231ms -> 2310
#define MAX_I 1000000
#define Ku 0.085
#define Tu 2310.0
#define T_PID (float)PID_interval
#define Kp 0.6*Ku //0.6
#define Ki 2.1*Kp*T_PID/Tu //2.0
#define Kd 0.14*Kp*Tu/T_PID //0.12
extern float errorlast;
extern float iterm;

// zmienne pomocnicze do regulatorow
extern float errorlastwallrot;
extern float itermwallrot;
extern float errorlastwalltrans;
extern float itermwalltrans;
extern float wallPcalibrot;
extern float wallPcalibtrans;
extern float errorlastenc;
extern float itermenc;
extern float wallBcalib;

extern float wallBcalibL;
extern float wallBcalibR;

enum statewall {W_LEFT, W_RIGHT};
extern uint8_t walllast;

extern float destLF, destRF, speedTrans, speedRot, itermL, itermR;
extern int32_t destL;
extern int32_t destR;
extern int32_t errorlastL;
extern int32_t errorlastR;

// definicje do latwego wlaczania i wylaczania diod od scian
#define S_BL ADC_Channel_15
#define S_BR ADC_Channel_10
#define S_PL ADC_Channel_14
#define S_PR ADC_Channel_11
#define S_SL ADC_Channel_13
#define S_SR ADC_Channel_12

#define BR_ON	GPIO_WriteBit(GPIOA, GPIO_Pin_4, 1)
#define BR_OFF	GPIO_WriteBit(GPIOA, GPIO_Pin_4, 0)
#define PR_ON	GPIO_WriteBit(GPIOA, GPIO_Pin_5, 1)
#define PR_OFF	GPIO_WriteBit(GPIOA, GPIO_Pin_5, 0)
#define SR_ON	GPIO_WriteBit(GPIOA, GPIO_Pin_6, 1)
#define SR_OFF	GPIO_WriteBit(GPIOA, GPIO_Pin_6, 0)
#define SL_ON	GPIO_WriteBit(GPIOA, GPIO_Pin_7, 1)
#define SL_OFF	GPIO_WriteBit(GPIOA, GPIO_Pin_7, 0)
#define BL_ON	GPIO_WriteBit(GPIOB, GPIO_Pin_10, 1)
#define BL_OFF	GPIO_WriteBit(GPIOB, GPIO_Pin_10, 0)
#define PL_ON	GPIO_WriteBit(GPIOB, GPIO_Pin_11, 1)
#define PL_OFF	GPIO_WriteBit(GPIOB, GPIO_Pin_11, 0)

#define BR 0
#define PR 1
#define SR 2
#define SL 3
#define BL 4
#define PL 5



// prototypy funkcji
void UART_Init(void);
void UART_Send(void);
void SENSORS_Init(void);
void OLED_Init(void);
void OLED_Draw(void);
inline void OLED_Refresh(void);
void OLED_buff_LiPo(void);
uint16_t readSensor(uint8_t channel);
void HBridge_Init(void);
void readSP(void);
void readSB(void);
void readSS(void);
void Encoders_Init(void);
void PID_Init(void);
void setMotors(int32_t pR, int32_t pL);
void reportbattery(void);
void PID_On(void);
void PID_Off(void);
void clamp_iterm(float *term, float max);
void avgcalc(void);
void calibrate_gyro(void);
void forward(uint8_t n);
void TIM3_IRQHandler(void);

#endif /* DEFINICJE_H_ */
