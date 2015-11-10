#include <stddef.h>
#include "stm32f10x.h"
#include "u8g_arm.h"
#include "lsm330.h"
#include "definicje.h"
#include "algorytm.h"

//------ DEBUG
uint8_t send=0;
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
	OLED_Init();
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
	// joystick
	Joystick_Init();


	reportbattery();
	sprintf(buffer, "Restarted\n\r");
	UART_Send();


	// -------------
	uint32_t mi = 0, mj = 0;

	while (1)
	{
		//podczas jazdy:
		if(!oled_state) {
			// odczytaj czujniki
			if(sensors_state) {
				readSP();
				//readSS();
				readSB();
				avgcalc();
			}
			/*if(send) {
				//sprintf(buffer, "err*1000: %d\n\rgyroint: %d\n\r", (int32_t)(errorlastwallrot*1000.0), (int32_t)gyroint);
				sprintf(buffer, "BL: %d\n\rBR: %d\n\r", (int32_t)((avgBL)*1000.0), (int32_t)((avgBR)*1000.0));
				UART_Send();
				send=0;
			}*/

			mi=3000;
			while(mi--);

			mj++;
		}
		// a to podczas uzywania OLEDa:
		else {
			sprintf(buffer1, "GORA %d", UP_count);
			sprintf(buffer2, "DOL %d", DOWN_count);
			sprintf(buffer3, "LEWO %d", LEFT_count);
			sprintf(buffer4, "PRAWO %d", RIGHT_count);
			sprintf(buffer5, "OK %d", OK_count);
			sprintf(buffer6, "T %d", counter_2ms*2/1000);
			//sprintf(buffer6, "Gz %d", LSM330_GetGyroZ());

			// odswiezenie wyswietlacza
			OLED_buff_LiPo();
			OLED_Refresh();
		}
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

        if(data=='g') {sensors_state=1; calibrate_gyro(); sprintf(buffer, "GyroAvg: %d\n\r", (int)gyroavg); UART_Send();}
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
	u8g_DrawStr(&u8g, 1, 8, "l");
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
// Przerwanie 500Hz
void TIM3_IRQHandler(void) {
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

		counter_2ms++;
		if(joystick_debounce1>0) joystick_debounce1--;

		//czasem przerwanie tylko odlicza czas:
		if(PID_enabled) {

			//wylaczenie diod jakby co:
			char statePL = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_11);
			char statePR = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_5);
			char stateBL = GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_10);
			char stateBR = GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_4);
			PL_OFF;
			PR_OFF;
			BL_OFF;
			BR_OFF;

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


			//przywrĂ�Âłcenie stanu diod:
			GPIO_WriteBit(GPIOB, GPIO_Pin_11, statePL);
			GPIO_WriteBit(GPIOA, GPIO_Pin_5, statePR);
			GPIO_WriteBit(GPIOB, GPIO_Pin_10, stateBL);
			GPIO_WriteBit(GPIOA, GPIO_Pin_4, stateBR);
		}
	}
}

void JOY_UP(void) {
	UP_count++;
}
void JOY_DOWN(void) {
	DOWN_count++;
}
void JOY_LEFT(void) {
	LEFT_count++;
}
void JOY_RIGHT(void) {
	RIGHT_count++;
}
