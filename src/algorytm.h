#ifndef ALGORYTM_H_
#define ALGORYTM_H_

#include <stdint.h>


#define polnoc 1
#define wschod 2
#define poludnie 3
#define zachod 4

extern uint8_t sensor_gora[16][16];
extern uint8_t sensor_prawo[16][16];
extern uint8_t sensor_dol[16][16];
extern uint8_t sensor_lewo[16][16];
extern int mapa[16][16];
extern int droga[256][2];
extern int sciezka[256];
//int kierunekTab[16][16];
int i; //wiersz
int j; //kolumna
extern int kierunek; //kierunek przodu robota
extern int czy_byl[16][16];
int a;
int b;
int l;
int d;
extern uint8_t sensor_front;
extern uint8_t sensor_right;
extern uint8_t sensor_left;

void zalewanie(void);
//void zalewanieCzasem(void);
//void zalewanieCzasem_2(void);
void zalewanieBlokow(int ii, int jj);
uint8_t nextscanstep(void);
int gdzie_jechac(void);
void obrobkaSensorow(void);
void czy_i_gdzie_jechac(void);
void sciezkaFun(void);
void jaka_nastepna_komorka(void);
void sensory_zamiana(void);
void mazeinit(void);
void maze_reinit(void);
void szybko(void);


#endif /* ALGORYTM_H_ */
