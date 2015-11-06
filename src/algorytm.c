/*
 * algorytm.c
 *
 *  Created on: 27 paü 2015
 */
#include <stdint.h>
#include "algorytm.h"
#include "definicje.h"

uint8_t sensor_gora[16][16]={0};
uint8_t sensor_prawo[16][16]={0};
uint8_t sensor_dol[16][16]={0};
uint8_t sensor_lewo[16][16]={0};
int czy_byl[16][16];
int droga[256][2];
int mapa[16][16];
int sciezka[256];
int czy_byl[16][16];
char buffer[50];
int kierunek = polnoc; //kierunek przodu robota
int i = 0; //wiersz
int j = 0; //kolumna
int a=0;
int b=0;
int l=0;
int d=0;
uint8_t sensor_front;
uint8_t sensor_right;
uint8_t sensor_left;

// --------------------------------------------- ALGORYTM
void mazeinit()
{

	int16_t y;
	int16_t x;
	for (y=0; y < 16; y++)
	{
		for (x=0; x < 16; x++)
		{
			sensor_gora[x][y] = 0;
			sensor_lewo[x][y] = 0;
			sensor_dol[x][y] = 0;
			sensor_lewo[x][y] = 0;
			czy_byl[x][y] = 0;
			mapa[x][y] = 0;

		}
	}

	for (x=0; x < 16; x++)
	{
		sensor_gora[15][x] = 1;
		sensor_prawo[x][15] = 1;
		sensor_dol[0][x] = 1;
		sensor_lewo[x][0] = 1;
	}

	for (x = 0; x < 256; x++)
	{
		droga[x][0] = -1;
		droga[x][1] = -1;
		sciezka[x] = 0;
	}

	droga[0][0] = 0;
	droga[0][1] = 0;
	czy_byl[0][0] = 1;

	kierunek=polnoc;

	obrobkaSensorow();
	zalewanie(); // pirwsze zalanie labiryntu

	i=0;
	j=0;
	a=0;
	b=0;
	l=0;
	d=0;
}

void maze_reinit()
{

	int16_t y;
	int16_t x;
	for (y=0; y < 16; y++)
	{
		for (x=0; x < 16; x++)
		{
			mapa[x][y] = 0;

		}
	}

	for (x = 0; x < 256; x++)
	{
		droga[x][0] = -1;
		droga[x][1] = -1;
	}

	droga[0][0] = 0;
	droga[0][1] = 0;

	kierunek=polnoc;

	obrobkaSensorow();
	zalewanie(); // pirwsze zalanie labiryntu

	i=0;
	j=0;
	a=0;
	b=0;
	l=0;
	d=0;
}

void zalewanie()
{
	int x;
	for (x = 0; x < 16; x++)
	{
		int y;
		for (y = 0; y < 16; y++)
		{
			mapa[x][y] = -1;
		}
	}

	mapa[7][7] = 0;
	mapa[7][8] = 0;
	mapa[8][7] = 0;
	mapa[8][8] = 0;

	int p = 1;
	int k;
	int u;

	int iteracje=0;

	while (p && iteracje<1000)
	{
		p = 0;
		int y;
		for (y = 0; y < 8; y++)
		{
			k = 15 - y;
			int x;
			for (x = 0; x < 8; x++)
			{
				zalewanieBlokow(x, y);
				zalewanieBlokow(x, k);
				u = 15 - x;
				zalewanieBlokow(u, y);
				zalewanieBlokow(u, k);
				if (mapa[x][y] == -1 || mapa[x][k] == -1 || mapa[u][y] == -1 || mapa[u][k] == -1)
				{
					p = 1;
				}
			}
		}


		iteracje++;
	}

	sprintf(buffer, "Iter: %d\n\r", iteracje);
	UART_Send();
}

void zalewanieBlokow(int ii, int jj){
	if (ii != 15){
		if (sensor_gora[ii][jj] == 0 && mapa[ii + 1][jj] == -1 && mapa[ii][jj] != -1){
			mapa[ii + 1][jj] = mapa[ii][jj] + 1;
		}
	}
	if (jj != 15){
		if (sensor_prawo[ii][jj] == 0 && mapa[ii][jj + 1] == -1 && mapa[ii][jj] != -1){
			mapa[ii][jj + 1] = mapa[ii][jj] + 1;
		}
	}
	if (ii != 0){
		if (sensor_dol[ii][jj] == 0 && mapa[ii - 1][jj] == -1 && mapa[ii][jj] != -1){
			mapa[ii - 1][jj] = mapa[ii][jj] + 1;
		}
	}
	if (jj != 0){
		if (sensor_lewo[ii][jj] == 0 && mapa[ii][jj - 1] == -1 && mapa[ii][jj] != -1){
			mapa[ii][jj - 1] = mapa[ii][jj] + 1;
		}
	}
}

int gdzie_jechac()
{
	a = 0;
	b = 0;
	if (i != 15){
		if (mapa[i+1][j]<mapa[i][j] && sensor_gora[i][j] == 0){
			a = i+1;
			b = j;
		}
	}
	if (j != 15){
		if (mapa[i][j+1]<mapa[i][j] && sensor_prawo[i][j] == 0){
			a = i;
			b = j + 1;
		}
	}
	if (i != 0){
		if (mapa[i-1][j]<mapa[i][j] && sensor_dol[i][j] == 0){
			a = i-1;
			b = j;
		}
	}
	if (j != 0){
		if (mapa[i][j-1]<mapa[i][j] && sensor_lewo[i][j] == 0){
			a = i;
			b = j-1;
		}
	}
	if (czy_byl[a][b]==0)
	{
		return 0;
	}else{
		return 1;
	}
}

void obrobkaSensorow()
{
	int x;
	for (x = 0; x<16; x++){
		int y;
		for (y = 0; y<16; y++){
			if (x != 15){
				if (sensor_gora[x][y] == 1){
					sensor_dol[x+1][y] = 1;
				}
			}
			if (y != 15){
				if (sensor_prawo[x][y] == 1){
					sensor_lewo[x][y+1] = 1;
				}
			}
			if (x != 0){
				if (sensor_dol[x][y] == 1){
					sensor_gora[x-1][y] = 1;
				}
			}
			if (y != 0){
				if (sensor_lewo[x][y] == 1){
					sensor_prawo[x][y-1] = 1;
				}
			}
		}
	}
}

void czy_i_gdzie_jechac()
{
	l=0;
	if (i != 15){
		if (mapa[i+1][j]<mapa[i][j] && sensor_gora[i][j] == 0){
			l = 1;
			a = i+1;
			b = j;
		}
	}
	if (j != 15){
		if (mapa[i][j+1]<mapa[i][j] && sensor_prawo[i][j] == 0){
			l = 1;
			a = i;
			b = j+1;
		}
	}
	if (i != 0){
		if (mapa[i-1][j]<mapa[i][j] && sensor_dol[i][j] == 0){
			l = 1;
			a = i-1;
			b = j;
		}
	}
	if (j != 0){
		if (mapa[i][j-1]<mapa[i][j] && sensor_lewo[i][j] == 0){
			l = 1;
			a = i;
			b = j-1;
		}
	}
}

uint8_t nextscanstep(void) {
		int w, e, dd, k;

		sensor_front=(wallPcalibtrans-(avgshortPR+avgshortPL))<33.0;
		sensor_left=avgshortBL>30.0;
		sensor_right=avgshortBR>30.0;

		sprintf(buffer, "0");
		UART_Send();
		sensory_zamiana();
		sprintf(buffer, "1");
		UART_Send();
		obrobkaSensorow();
		sprintf(buffer, "2");
		UART_Send();
		czy_i_gdzie_jechac();
		sprintf(buffer, "3\n\r");
		UART_Send();

		//sprintf(buffer, "sg: %u, sd: %u, sp: %u, sl: %u\n\r", sensor_gora[0][0], sensor_dol[0][0], sensor_prawo[0][0], sensor_lewo[0][0]);
		//sprintf(buffer, "m00: %u, m10: %u, m01: %u\n\r", mapa[0][0], mapa[1][0], mapa[0][1]);
		//sprintf(buffer, "a: %u, b: %u\n\r", a, b);
		sprintf(buffer, "----- SKAN\n\rsf: %u, sl: %u, sr: %u, kier: %d, pi: %d, pj: %d, l: %d\n\r", sensor_front, sensor_left, sensor_right, kierunek, i, j, l);
		UART_Send();

		if (l == 1) {
			jaka_nastepna_komorka();
			w = i;
			e = j;
			i = a;
			j = b;
		} else {
			zalewanie();
			czy_i_gdzie_jechac();
			jaka_nastepna_komorka();
			w = i;
			e = j;
			i = a;
			j = b;
		}
		czy_byl[i][j] = 1;


		sprintf(buffer, "Ruch: %c\n\r", (moves[0]==M_F1)?'F':((moves[0]==M_L)?'L':((moves[0]==M_R)?'R':'O')));
		UART_Send();

		if(mapa[w][e]!=0)
		{
			return 1;
		}else
		{
			i=0;
			j=0;
			dd=1;
			k=1;
			while (mapa[i][j]!=0)
			{
				dd=gdzie_jechac();
				if (dd==0)
				{
					break;
				}
				i=a;
				j=b;
				droga[k][0]=i;
				droga[k][1]=j;
				k=k+1;
			}


			if(dd==0)
			{
				sprintf(buffer, "\n\rNakurwiamy dalej!\n\r");
				UART_Send();
			}else{
				sprintf(buffer, "\n\rWystarczy!\n\r");
				UART_Send();

				sciezkaFun();
				szybko();
				int x;
				for(x=0; x<100; x++) {
					sprintf(buffer, "M[%d]=%d - %d\n\r", x, droga[x][0], droga[x][1]);
					UART_Send();
				}
			}

			return 0;
		}
}

void jaka_nastepna_komorka()
{

	switch(kierunek) {
	case polnoc:
		if (a > i)
		{
			moves[0] = M_F1;
			moves[1] = M_SCAN;
		} else if (a < i)
		{
			moves[0] = M_L;
			moves[1] = M_L;
			moves[2] = M_F1;
			moves[3] = M_SCAN;
			kierunek = poludnie;
		} else if (b > j)
		{
			moves[0] = M_R;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = wschod;
		} else if(b<j)
		{
			moves[0] = M_L;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = zachod;
		}
		break;
	case wschod:
		if (a > i)
		{
			moves[0] = M_L;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = polnoc;
		} else if (a < i)
		{
			moves[0] = M_R;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = poludnie;
		} else if (b > j)
		{
			moves[0] = M_F1;
			moves[1] = M_SCAN;
		} else if (b < j)
		{
			moves[0] = M_L;
			moves[1] = M_L;
			moves[2] = M_F1;
			moves[3] = M_SCAN;
			kierunek = zachod;
		}
		break;
	case poludnie:
		if (a > i)
		{
			moves[0] = M_L;
			moves[1] = M_L;
			moves[2] = M_F1;
			moves[3] = M_SCAN;
			kierunek = polnoc;
		} else if (a < i)
		{
			moves[0] = M_F1;
			moves[1] = M_SCAN;
		} else if (b > j)
		{
			moves[0] = M_L;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = wschod;
		} else if (b<j)
		{
			moves[0] = M_R;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = zachod;
		}
		break;
	case zachod:
		if (a > i)
		{
			moves[0] = M_R;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = polnoc;
		} else if (a < i)
		{
			moves[0] = M_L;
			moves[1] = M_F1;
			moves[2] = M_SCAN;
			kierunek = poludnie;
		} else if (b > j)
		{
			moves[0] = M_L;
			moves[1] = M_L;
			moves[2] = M_F1;
			moves[3] = M_SCAN;
			kierunek = wschod;
		} else if (b < j)
		{
			moves[0] = M_F1;
			moves[1] = M_SCAN;
		}
		break;
	}
}

void sciezkaFun()
{

	kierunek= polnoc;
	int v = 0;
	int count=0;

	while (droga[v+1][0] != -1 && droga[v+1][1] != -1)
	{
		switch (kierunek)
		{
		case polnoc:
			if (droga[v + 1][1] > droga[v][1])
			{
				kierunek= wschod;
				sciezka[count] = 2;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;
			}
			else if (droga[v + 1][1] < droga[v][1])
			{
				kierunek= zachod;
				sciezka[count] = 3;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;

			} else
			{
				sciezka[count] = 1;
				v = v + 1;
				count++;
			}
			break;

		case wschod:
			if (droga[v + 1][0] > droga[v][0])
			{
				kierunek= polnoc;
				sciezka[count] = 3;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;
			}
			else if (droga[v + 1][0] < droga[v][0])
			{
				kierunek= poludnie;
				sciezka[count] = 2;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;

			} else
			{
				sciezka[count] = 1;
				v = v + 1;
				count++;
			}
			break;

		case poludnie:
			if (droga[v + 1][1] > droga[v][1])
			{
				kierunek= wschod;
				sciezka[count] = 3;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;

			}
			else if (droga[v + 1][1] < droga[v][1])
			{
				kierunek= zachod;
				sciezka[count] = 2;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;

			} else
			{
				sciezka[count] = 1;
				v = v + 1;
				count++;
			}
			break;

		case zachod:
			if (droga[v + 1][0] > droga[v][0])
			{
				kierunek= polnoc;
				sciezka[count] = 2;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;
			}
			else if (droga[v + 1][0] < droga[v][0])
			{
				kierunek= poludnie;
				sciezka[count] = 3;
				count++;
				sciezka[count] = 1;
				count++;
				v = v + 1;
			} else
			{
				sciezka[count] = 1;
				v = v + 1;
				count++;
			}
			break;
		}

	}
}


void sensory_zamiana()
{

	switch (kierunek)
	{
	case polnoc:
		sensor_gora[i][j] = sensor_front;
		sensor_prawo[i][j] = sensor_right;
		sensor_lewo[i][j] = sensor_left;
		break;
	case wschod:
		sensor_gora[i][j] = sensor_left;
		sensor_prawo[i][j] = sensor_front;
		sensor_dol[i][j] = sensor_right;
		break;
	case poludnie:
		sensor_prawo[i][j] = sensor_left;
		sensor_dol[i][j] = sensor_front;
		sensor_lewo[i][j] = sensor_right;
		break;
	case zachod:
		sensor_gora[i][j] = sensor_right;
		sensor_dol[i][j] = sensor_left;
		sensor_lewo[i][j] = sensor_front;
		break;
	}
}
void szybko() {
	int dlpr = 0;
	int x;
	int y;
	int z=0;

	for(x=0;x<254;x++){
		if(sciezka[x]==1){
			dlpr=1;
			for(y=x;y<254;y++){
				if(sciezka[y+1]==1 && sciezka[y]==1) {
					dlpr++;
				}
				else{
					break;
				}
			}
			if(dlpr==1){moves[z]=M_F1;}
			else if(dlpr==2){moves[z]=M_F2;}
			else if(dlpr==3){moves[z]=M_F3;}
			else if(dlpr==4){moves[z]=M_F4;}
			else if(dlpr==5){moves[z]=M_F5;}
			else if(dlpr==6){moves[z]=M_F6;}
			else if(dlpr==7){moves[z]=M_F7;}
			else if(dlpr==8){moves[z]=M_F8;}
			else if(dlpr==9){moves[z]=M_F9;}
			else if(dlpr==10){moves[z]=M_F10;}
			else if(dlpr==11){moves[z]=M_F11;}
			else if(dlpr==12){moves[z]=M_F12;}
			else if(dlpr==13){moves[z]=M_F13;}
			else if(dlpr==14){moves[z]=M_F14;}
			else if(dlpr==15){moves[z]=M_F15;}
		}
		else if(sciezka[x]==2){
			moves[z]=M_R;
			x++;
		}
		else if(sciezka[x]==3){
			moves[z]=M_L;
			x++;
		}
		else{
			moves[z]=M_STOP;
			return;
		}

		z++;
		x=x+dlpr-1;
		dlpr=0;
	}
}

/*void zalewanieCzasem()
{
	int x,y;
	for (y = 0; y < 16; y++)
	{
		for (x = 0; x < 16; x++)
		{
			mapa[x][y] = -1;
			kierunekTab[x][y] = 0;
		}
	}
	mapa[7][7] = 0;
	mapa[7][8] = 0;
	mapa[8][7] = 0;
	mapa[8][8] = 0;

	kierunekTab[7][7] = 4;
	kierunekTab[7][8] = 3;
	kierunekTab[8][8] = 2;
	kierunekTab[8][7] = 1;

	int skret = 3;
	int p = 1;

	while (p)
	{
		p = 0;
		for (y = 0; y < 16; y++)
		{
			for (x = 0; x < 16; x++)
			{
				if (kierunekTab[x][y] == 0)
				{
					if (x != 15)
					{
						if (sensor_gora[x][y] == 0 && kierunekTab[x + 1][y] != 0)
						{
							if (kierunekTab[x + 1][y] == 3)
							{
								mapa[x][y] = mapa[x + 1][y] + 1;
								kierunekTab[x][y] = 3;
							} else
							{
								mapa[x][y] = mapa[x + 1][y] + skret;
								kierunekTab[x][y] = 3;
							}
						}
					}

					if (y != 15)
					{
						if (sensor_prawo[x][y] == 0 && kierunekTab[x][y + 1] != 0)
						{
							if (kierunekTab[x][y + 1] == 4)
							{
								mapa[x][y] = mapa[x][y + 1] + 1;
								kierunekTab[x][y] = 4;
							} else
							{
								mapa[x][y] = mapa[x][y + 1] + skret;
								kierunekTab[x][y] = 4;
							}
						}
					}
					if (x != 0)
					{
						if (sensor_dol[x][y] == 0 && kierunekTab[x - 1][y] != 0)
						{
							if (kierunekTab[x - 1][y] == 1)
							{
								mapa[x][y] = mapa[x - 1][y] + 1;
								kierunekTab[x][y] = 1;
							} else
							{
								mapa[x][y] = mapa[x - 1][y] + skret;
								kierunekTab[x][y] = 1;
							}
						}
					}
					if (y != 0)
					{
						if (sensor_lewo[x][y] == 0 && kierunekTab[x][y - 1] != 0)
						{
							if (kierunekTab[x][y - 1] == 2)
							{
								mapa[x][y] = mapa[x][y - 1] + 1;
								kierunekTab[x][y] = 2;
							} else
							{
								mapa[x][y] = mapa[x][y - 1] + skret;
								kierunekTab[x][y] = 2;
							}
						}
					}
				}

				if (mapa[x][y] == -1)
				{
					p = 1;
				}
			}
		}
	}
}

void zalewanieCzasem_2()
{
	int x, y;
	for (y = 0; y < 16; y++)
	{
		for (x = 0; x < 16; x++)
		{
			mapa[x][y] = -1;
			kierunekTab[x][y] = 0;
		}
	}
	mapa[7][7] = 0;
	mapa[7][8] = 0;
	mapa[8][7] = 0;
	mapa[8][8] = 0;

	kierunekTab[7][7] = 8;
	kierunekTab[7][8] = 8;
	kierunekTab[8][8] = 8;
	kierunekTab[8][7] = 8;

	if (sensor_dol[7][7]==0)
	{
		kierunekTab[6][7]=3;
		mapa[6][7]=1;
	}
	if (sensor_dol[7][8]==0)
	{
		kierunekTab[6][8]=3;
		mapa[6][8]=1;
	}
	if (sensor_prawo[7][8]==0)
	{
		kierunekTab[7][9]=2;
		mapa[7][9]=1;
	}
	if (sensor_prawo[8][8]==0)
	{
		kierunekTab[8][9]=2;
		mapa[8][9]=1;
	}
	if (sensor_gora[8][7]==0)
	{
		kierunekTab[9][7]=1;
		mapa[9][7]=1;
	}
	if (sensor_gora[9][8]==0)
	{
		kierunekTab[7][8]=1;
		mapa[9][8]=1;
	}
	if (sensor_lewo[8][7]==0)
	{
		kierunekTab[8][6]=4;
		mapa[8][6]=1;
	}
	if (sensor_lewo[7][7]==0)
	{
		kierunekTab[7][6]=4;
		mapa[7][6]=1;
	}


	int skret = 3;
	int p, t;

	for (t = 0; t < 256; t++)
	{
		p = 0;
		for (y = 0; y < 16; y++)
		{
			for (x = 0; x < 16; x++)
			{
				if (x != 15)
				{
					if (kierunekTab[x][y] == 0 && sensor_gora[x][y] == 0 && kierunekTab[x + 1][y] != 0
							|| sensor_gora[x][y] == 0 && (mapa[x][y] - mapa[x + 1][y]) > 1 && (mapa[x][y] - mapa[x + 1][y]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x + 1][y] != 0)
					{
						if (kierunekTab[x + 1][y] == 3)
						{
							mapa[x][y] = mapa[x + 1][y] + 1;
							kierunekTab[x][y] = 3;
						} else
						{
							mapa[x][y] = mapa[x + 1][y] + skret;
							kierunekTab[x][y] = 3;
						}
						p = 1;
					}
				}

				if (y != 15)
				{
					if (kierunekTab[x][y] == 0 && sensor_prawo[x][y] == 0 && kierunekTab[x][y + 1] != 0
							|| sensor_prawo[x][y] == 0 && (mapa[x][y] - mapa[x][y + 1]) > 1 && (mapa[x][y] - mapa[x][y + 1]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x][y + 1] != 0)
					{
						if (kierunekTab[x][y + 1] == 4)
						{
							mapa[x][y] = mapa[x][y + 1] + 1;
							kierunekTab[x][y] = 4;
						} else
						{
							mapa[x][y] = mapa[x][y + 1] + skret;
							kierunekTab[x][y] = 4;
						}
						p = 1;
					}
				}
				if (x != 0)
				{
					if (kierunekTab[x][y] == 0 && sensor_dol[x][y] == 0 && kierunekTab[x - 1][y] != 0
							|| sensor_dol[x][y] == 0 && (mapa[x][y] - mapa[x - 1][y]) > 1 && (mapa[x][y] - mapa[x - 1][y]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x - 1][y] != 0)
					{
						if (kierunekTab[x - 1][y] == 1)
						{
							mapa[x][y] = mapa[x - 1][y] + 1;
							kierunekTab[x][y] = 1;
						} else
						{
							mapa[x][y] = mapa[x - 1][y] + skret;
							kierunekTab[x][y] = 1;
						}
						p = 1;
					}
				}
				if (y != 0)
				{
					if (kierunekTab[x][y] == 0 && sensor_lewo[x][y] == 0 && kierunekTab[x][y - 1] != 0
							|| sensor_lewo[x][y] == 0 && (mapa[x][y] - mapa[x][y - 1]) > 1 && (mapa[x][y] - mapa[x][y - 1]) != skret && kierunekTab[x][y] != 0 && kierunekTab[x][y - 1] != 0)
					{
						if (kierunekTab[x][y - 1] == 2)
						{
							mapa[x][y] = mapa[x][y - 1] + 1;
							kierunekTab[x][y] = 2;
						} else
						{
							mapa[x][y] = mapa[x][y - 1] + skret;
							kierunekTab[x][y] = 2;
						}
						p = 1;
					}
				}
			}
		}
		if (p == 0)
		{
			break;
		}
	}
}*/

