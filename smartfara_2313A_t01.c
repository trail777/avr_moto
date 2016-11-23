/*
 * moto1.c
 *
 * Created: 04.07.2016 9:33:22
 * Author: Gleb
 
 *********** ATtiny 2313a **********
 Fuses:
   Lock Bits: mode 1
   SPIEN=0;
   BODLEVEL=111;  //101-2.7В; 100-4.4В
   CKOUT=0 (Clock output on PORTD2)
   Int RC Osc. 4MHz; Start-up time: 14CK+65ms(CKSEL=0010)
   ******************************************************
PORTA (bits "RESET", "XTAL1", "XTAL2") by default
 ****  PORT PA0,PA1   //Тактирование от 4МГц другого МК
 ****  PORT PA2       //RESET   (внешняя подтяжка)

 ////////// PORTB /////////
 ****  PORT PB0 --Детектирование правого поворотника (вход с внешней подтяжкой)
 ****  PORT PB1 --Детектирование левого поворотника  (вход с внешней подтяжкой)
 ****  PORT PB2 --Переключатель поз.3                (вход с внешней подтяжкой)
 ****  PORT PB3 --Переключатель поз.2                (вход с внешней подтяжкой)
 ****  PORT PB4 --Переключатель поз.1                (вход с внешней подтяжкой)
 ****  PORT PB5 --R3 (+MOSI input c подтяжкой)                   (Выход)
 ****  PORT PB6 --R2 (+MISO)                                     (Выход)  
 ****  PORT PB7 --R1 (SCK input с подтяжкой)                     (Выход)
 
 ////////// PORTD ///////// (нач. init: DDRD=PORTD=0xFF)
 ****  PORT PD0 -- L1       (выход)
 ****  PORT PD1 -- L2       (выход)
 ****  PORT PD3 -- L3       (выход)
 ****  PORT PD4 -- L4       (выход)
 ***** PORT PD2 - выход тактирования ATiny13
 ***** PORT PD5 --Габариты (выход)
 ***** PORT PD6 --R4       (выход)
 */ 
/////////////////////////////////////////////////////////
//********* ATtiny 13a (выставить фьюз CKSEL=0 внешнее тактирование, подается на PB3)
 ////////// PORTB /////////
 //****  PORT PB0 - не задействован
 //****  PORT PB1 --Детектирование Стоп-сигнала        (вход с внешней подтяжкой)
 //****  PORT PB2 --Переключатель поз.4                (вход с внешней подтяжкой)
 //****  PORT PB3 --вход тактирования со стороны ATTiny2313a
 //****  PORT PB4 --Стоп-сигнал (выход)
 //****  PORT PB5 --RESET   (внешняя подтяжка)
 
 //////////////////////////////////////////
 //   Switch     1      2      3   mode   
 //             PB4    PB3    PB2
 //              0      0      0     0
 //              1      0      0     1
 //              0      1      0     2
 //              1      1      0     3
 //              0      0      1     4
 //              1      0      1     5
 //              0      1      1     6
 //              1      1      1     7
 // **************************************
#ifndef F_CPU
#define F_CPU 4000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
 
volatile unsigned int num_timer1;
volatile unsigned char flag_detect_pos_R, flag_detect_pos_L;
unsigned int mode;

ISR (TIMER1_COMPA_vect);
ISR (PCINT0_vect);
void pause_t1_ms(unsigned int interval_ms);
void left_itr_0(void), left_itr_1(void), left_itr_2(void), left_itr_3(void), left_itr_4(void), left_itr_5(void), left_itr_6(void), left_itr_7(void);
void right_itr_0(void), right_itr_1(void), right_itr_2(void), right_itr_3(void), right_itr_4(void), right_itr_5(void), right_itr_6(void), right_itr_7(void);
void (*arr_pf_left_itteration []) (void) = { //инициализиция массива указателей на функции левого поворотника
	left_itr_0,   //обычный левый поворотник           // 0 0 0 
	left_itr_1,   //бегущий огонь с заполнением        // 1 0 0
	left_itr_2,                                   //010
	left_itr_3,									  //110
	left_itr_4,									  //001	
	left_itr_5,								      //101
	left_itr_6,                                   //011
	left_itr_7   // просто бегущий огонь              // 1 1 1 
};
void (*arr_pf_right_itteration []) (void) = { //инициализиция массива указателей на функции правого поворотника
	right_itr_0,   //обычный правый поворотник        // 0 0 0
	right_itr_1,   //бегущий огонь с заполнением      // 1 0 0
	right_itr_2,
	right_itr_3,
	right_itr_4,
	right_itr_5,
	right_itr_6,
	right_itr_7   // просто бегущий огонь             // 1 1 1
};
void Handler_left_detection(int); 
void Handler_right_detection(int); 
unsigned char read_mode(void);
void CHECK_SET_START_CONDITION (void); 

int arrL[4]={0,1,3,4}; //выводы порта D для управления левым повортником
int arrR[3]={7,6,5};   //выводы порта B и (PD6)

int main(void)
{
    //// Отключаем неиспользуемую перефирию
	PRR|=(1<<PRTIM0)|(1<<PRUSI)|(1<<PRUSART); // Disabled T0, USI, USART
	
	////INIT ports //////////////////////////
	DDRB=0b11100000; PORTB=0b11100000; DDRD=0xFF;PORTD=0b11011111; //Сразу включаем габариты
	asm volatile( "nop \n" "nop \n" "nop \n");
	//////////////////////////////////////////////////////////////
	
	/// INIT ext interrupts PCINT0, PCIN1
	PCMSK0|=(1<<PCINT0)|(1<<PCINT1); // Enabled PINs PCINT0, PCINT1 
	GIMSK|=(1<<PCIE0);               //Enabled interrupt PCINT7..0 
	//////////////////////////////////////////////////////////////
	
	/// INIT SLEEP MODE ///////////////////
	MCUCR|=(1<<SM0); MCUCR&=~(1<<SM1);   //Power-down
	MCUCR|=(1<<SE);                      //Sleep_Enable 
	////////////////////////////////////////////////////////////
	
	//////////////////////// init timers1 (16bit) ////////////////
	cli(); OCR1AH=0x01;OCR1AL=0xF4; sei();        // В регистр сравнения заносим 1мс --> (4000000/8)*0,001=500;
	TIMSK|=1<<OCIE1A;                             // Разрешаем прерывание по совпадению	
	asm volatile( "nop \n" "nop \n" "nop \n");
	///////////////////////////////////////////////////////////////
	
	unsigned char mode=read_mode(); 
	CHECK_SET_START_CONDITION(); asm volatile( "nop \n" "nop \n" "nop \n"); //Если повортник был влкючен до запуска платы включаем запускаем функцию без прерывания 
	for (;;)
	{
	 Handler_left_detection(mode);   //         детектируем напряжение на левом поворотнике:
	                             //---на время  Uаб запускаем в цикле нужный режим. 
								 //---при снятии напряжения прерываем цикл - выходим из ф-и.
	 Handler_right_detection(mode);
	 __asm__ __volatile__ ( "sleep" "\n\t" :: ); //засыпаем	если нет Uаб на паворотниках
	// А здесь оказываемся после пробудки при включении поворотников
	} 
}


ISR (TIMER1_COMPA_vect)
{
	TCNT1H=0;TCNT1L=0; num_timer1++;
}

ISR (PCINT0_vect) //Включили повортник или выключили
{
	unsigned char i=0, count_L=0, count_R=0;
	while(i < 160) //	было i<16
	{
		if (!(PINB&(1<<1))) count_L++;
		if (!(PINB&(1<<0))) count_R++;
		i++;
	}
	if (count_L > 100) flag_detect_pos_L=1;  //Считаем что левый поворотник включен (есть 12 В) ////было i>10
	else flag_detect_pos_L=0;
	
	if (count_R > 100) flag_detect_pos_R=1;  //Считаем что правый поворотник включен (есть 12В) ////было i>10
	else flag_detect_pos_R=0;
}


void CHECK_SET_START_CONDITION (void) 
{
	unsigned char i=0, count_L=0, count_R=0;
	while(i < 160) //	i<16
	{
		if (!(PINB&(1<<1))) count_L++;
		if (!(PINB&(1<<0))) count_R++;
		i++;
	}
	if (count_L > 100) flag_detect_pos_L=1;  //Считаем что левый поворотник включен (есть 12 В) ////i>10
	else flag_detect_pos_L=0;
	
	if (count_R > 100) flag_detect_pos_R=1;  //Считаем что правый поворотник включен (есть 12В)
	else flag_detect_pos_R=0;
}


void pause_t1_ms(unsigned int interval_ms)
{
	TCNT1H=0;TCNT1L=0;
	num_timer1=0;
	TCCR1B|=(1<<CS11); //Prescaler=8-->F_CPU/8-->Запуск счетчика  
	while(1) { if(num_timer1==interval_ms) {TCCR1B&=~(1<<CS10|1<<CS11|1<<CS12);break;}}
}


void Handler_left_detection(int mode) 
{
	while (flag_detect_pos_L) (*arr_pf_left_itteration[mode])(); /* выполнение соотвествующей режиму `mode` функции */
}

void Handler_right_detection(int mode)
{
	while (flag_detect_pos_R) (*arr_pf_right_itteration[mode])(); /* выполнение соотвествующей режиму `mode` функции */
}

unsigned char read_mode(void)
{
	unsigned char result=0;
	if (!(PINB&(1<<4))) result|=(1<<0);
	if (!(PINB&(1<<3))) result|=(1<<1);
	if (!(PINB&(1<<2))) result|=(1<<2);
	return result;
}
////////////////////////////////////////////////////////////////////////
//обычный  поворотник                      // 0 0 0 
void left_itr_0 (void)
{
	PORTD&=~((1<<0)|(1<<1)|(1<<3)|(1<<4)); // ON
	pause_t1_ms(500);
	PORTD|=(1<<0)|(1<<1)|(1<<3)|(1<<4);    // OFF
	pause_t1_ms(500);
}
void right_itr_0 (void)
{
	PORTB&=~((1<<7)|(1<<6)|(1<<5));PORTD&=~(1<<6); // ON
	pause_t1_ms(500);
	PORTB|=(1<<7)|(1<<6)|(1<<5);PORTD|=(1<<6);    // OFF
	pause_t1_ms(500);
}
/////////////////////////////////////////////////////
//бегущий огонь с заполнением              // 1 0 0
void left_itr_1 (void)
{	
	int i=0;
	for(i=0;i<4;i++) {PORTD&=~(1<<arrL[i]);pause_t1_ms(150);}
	//pause_t1_ms(100);
	PORTD|=(1<<0)|(1<<1)|(1<<3)|(1<<4);    // OFF
	pause_t1_ms(400);
}
void right_itr_1 (void)
{
	int i=0;
	for(i=0;i<3;i++) {PORTB&=~(1<<arrR[i]);pause_t1_ms(150);} PORTD&=~(1<<6);pause_t1_ms(150);
	PORTB|=(1<<7)|(1<<6)|(1<<5);PORTD|=(1<<6);    // OFF
	pause_t1_ms(400);
}

////////////////////////////////////////////////////
//
void left_itr_2 (void)
{
	;
}
void right_itr_2 (void)
{
	;
}

/////////////////////////////////////////////////////
//
void left_itr_3 (void)
{
	;
}
void  right_itr_3 (void)
{
	;
}

//////////////////////////////////////////////////////
//
void left_itr_4 (void)
{
	;
}
void right_itr_4 (void)
{
	;
}

/////////////////////////////////////////////////////
//
void left_itr_5 (void)
{
	;
}
void right_itr_5 (void)
{
	;
}

/////////////////////////////////////////////////////
//
void left_itr_6 (void)
{
	;
}
void right_itr_6 (void)
{
	;
}

//////////////////////////////////////////////////////
//  просто бегущий огонь                   // 1 1 1
void left_itr_7(void)
{
	int i=0;
	unsigned char nibble_l=0, nibble_h=0;
		for(i=0;i<4;i++)
		{
			nibble_l=~(1<<arrL[i]);  // 11111110  // --бегущий ноль
			nibble_l&=0b00011011;    // 00011010 // сбрасываем все биты кроме индексных
			nibble_h=PORTD&11100100; // ***00*00 // здесь сбраываем индексные биты
			PORTD=nibble_h|nibble_l; // ***11 10 //
			pause_t1_ms(100);        // время горения блока LEDs
		}
		//PORTD&=~((1<<0)|(1<<1)|(1<<3)|(1<<4)); // ON
		//pause_t1_ms(100);
		PORTD|=(1<<0)|(1<<1)|(1<<3)|(1<<4); //OFF
		pause_t1_ms(400); // Поворотник отдыхает. Пауза между иттерациями.
}
void right_itr_7(void)
{
	PORTB&=~(1<<7);pause_t1_ms(100);        
	PORTB|=(1<<7);PORTB&=~(1<<6);pause_t1_ms(100);
	PORTB|=(1<<6);PORTB&=~(1<<5);pause_t1_ms(100);
	PORTB|=(1<<5);PORTD&=~(1<<6);pause_t1_ms(100);
	PORTD|=(1<<6);pause_t1_ms(400); // Поворотник отдыхает. Пауза между иттерациями.	
}