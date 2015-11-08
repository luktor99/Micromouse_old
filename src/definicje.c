// inicjalizacje zmiennych
#include <stdint.h>
#include "definicje.h"

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

char buffer[50], bufferLiPo1[14], bufferLiPo2[7], bufferCharge[20];
char bufferAX[20], bufferAY[20], bufferAZ[20], bufferGX[20], bufferGY[20], bufferGZ[20];
char buffer1[10], buffer2[10], buffer3[10], buffer4[10], buffer5[10], buffer6[10];
