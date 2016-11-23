/*
 * moto1.c
 *
 * Created: 04.07.2016 9:33:22
 * Author: Gleb
 
 *********** ATtiny 2313a **********
 Fuses:
   Lock Bits: mode 1
   SPIEN=0;
   BODLEVEL=111;  //101-2.7�; 100-4.4�
   CKOUT=0 (Clock output on PORTD2)
   Int RC Osc. 4MHz; Start-up time: 14CK+65ms(CKSEL=0010)
   ******************************************************
PORTA (bits "RESET", "XTAL1", "XTAL2") by default
 ****  PORT PA0,PA1   //������������ �� 4��� ������� ��
 ****  PORT PA2       //RESET   (������� ��������)

 ////////// PORTB /////////
 ****  PORT PB0 --�������������� ������� ����������� (���� � ������� ���������)
 ****  PORT PB1 --�������������� ������ �����������  (���� � ������� ���������)
 ****  PORT PB2 --������������� ���.3                (���� � ������� ���������)
 ****  PORT PB3 --������������� ���.2                (���� � ������� ���������)
 ****  PORT PB4 --������������� ���.1                (���� � ������� ���������)
 ****  PORT PB5 --R3 (+MOSI input c ���������)                   (�����)
 ****  PORT PB6 --R2 (+MISO)                                     (�����)  
 ****  PORT PB7 --R1 (SCK input � ���������)                     (�����)
 
 ////////// PORTD ///////// (���. init: DDRD=PORTD=0xFF)
 ****  PORT PD0 -- L1       (�����)
 ****  PORT PD1 -- L2       (�����)
 ****  PORT PD3 -- L3       (�����)
 ****  PORT PD4 -- L4       (�����)
 ***** PORT PD2 - ����� ������������ ATiny13
 ***** PORT PD5 --�������� (�����)
 ***** PORT PD6 --R4       (�����)
 */ 
/////////////////////////////////////////////////////////
//********* ATtiny 13a (��������� ���� CKSEL=0 ������� ������������, �������� �� PB3)
 ////////// PORTB /////////
 //****  PORT PB0 - �� ������������
 //****  PORT PB1 --�������������� ����-�������        (���� � ������� ���������)
 //****  PORT PB2 --������������� ���.4                (���� � ������� ���������)
 //****  PORT PB3 --���� ������������ �� ������� ATTiny2313a
 //****  PORT PB4 --����-������ (�����)
 //****  PORT PB5 --RESET   (������� ��������)
 
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
void (*arr_pf_left_itteration []) (void) = { //������������� ������� ���������� �� ������� ������ �����������
	left_itr_0,   //������� ����� ����������           // 0 0 0 
	left_itr_1,   //������� ����� � �����������        // 1 0 0
	left_itr_2,                                   //010
	left_itr_3,									  //110
	left_itr_4,									  //001	
	left_itr_5,								      //101
	left_itr_6,                                   //011
	left_itr_7   // ������ ������� �����              // 1 1 1 
};
void (*arr_pf_right_itteration []) (void) = { //������������� ������� ���������� �� ������� ������� �����������
	right_itr_0,   //������� ������ ����������        // 0 0 0
	right_itr_1,   //������� ����� � �����������      // 1 0 0
	right_itr_2,
	right_itr_3,
	right_itr_4,
	right_itr_5,
	right_itr_6,
	right_itr_7   // ������ ������� �����             // 1 1 1
};
void Handler_left_detection(int); 
void Handler_right_detection(int); 
unsigned char read_mode(void);
void CHECK_SET_START_CONDITION (void); 

int arrL[4]={0,1,3,4}; //������ ����� D ��� ���������� ����� �����������
int arrR[3]={7,6,5};   //������ ����� B � (PD6)

int main(void)
{
    //// ��������� �������������� ���������
	PRR|=(1<<PRTIM0)|(1<<PRUSI)|(1<<PRUSART); // Disabled T0, USI, USART
	
	////INIT ports //////////////////////////
	DDRB=0b11100000; PORTB=0b11100000; DDRD=0xFF;PORTD=0b11011111; //����� �������� ��������
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
	cli(); OCR1AH=0x01;OCR1AL=0xF4; sei();        // � ������� ��������� ������� 1�� --> (4000000/8)*0,001=500;
	TIMSK|=1<<OCIE1A;                             // ��������� ���������� �� ����������	
	asm volatile( "nop \n" "nop \n" "nop \n");
	///////////////////////////////////////////////////////////////
	
	unsigned char mode=read_mode(); 
	CHECK_SET_START_CONDITION(); asm volatile( "nop \n" "nop \n" "nop \n"); //���� ��������� ��� ������� �� ������� ����� �������� ��������� ������� ��� ���������� 
	for (;;)
	{
	 Handler_left_detection(mode);   //         ����������� ���������� �� ����� �����������:
	                             //---�� �����  U�� ��������� � ����� ������ �����. 
								 //---��� ������ ���������� ��������� ���� - ������� �� �-�.
	 Handler_right_detection(mode);
	 __asm__ __volatile__ ( "sleep" "\n\t" :: ); //��������	���� ��� U�� �� ������������
	// � ����� ����������� ����� �������� ��� ��������� ������������
	} 
}


ISR (TIMER1_COMPA_vect)
{
	TCNT1H=0;TCNT1L=0; num_timer1++;
}

ISR (PCINT0_vect) //�������� ��������� ��� ���������
{
	unsigned char i=0, count_L=0, count_R=0;
	while(i < 160) //	���� i<16
	{
		if (!(PINB&(1<<1))) count_L++;
		if (!(PINB&(1<<0))) count_R++;
		i++;
	}
	if (count_L > 100) flag_detect_pos_L=1;  //������� ��� ����� ���������� ������� (���� 12 �) ////���� i>10
	else flag_detect_pos_L=0;
	
	if (count_R > 100) flag_detect_pos_R=1;  //������� ��� ������ ���������� ������� (���� 12�) ////���� i>10
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
	if (count_L > 100) flag_detect_pos_L=1;  //������� ��� ����� ���������� ������� (���� 12 �) ////i>10
	else flag_detect_pos_L=0;
	
	if (count_R > 100) flag_detect_pos_R=1;  //������� ��� ������ ���������� ������� (���� 12�)
	else flag_detect_pos_R=0;
}


void pause_t1_ms(unsigned int interval_ms)
{
	TCNT1H=0;TCNT1L=0;
	num_timer1=0;
	TCCR1B|=(1<<CS11); //Prescaler=8-->F_CPU/8-->������ ��������  
	while(1) { if(num_timer1==interval_ms) {TCCR1B&=~(1<<CS10|1<<CS11|1<<CS12);break;}}
}


void Handler_left_detection(int mode) 
{
	while (flag_detect_pos_L) (*arr_pf_left_itteration[mode])(); /* ���������� �������������� ������ `mode` ������� */
}

void Handler_right_detection(int mode)
{
	while (flag_detect_pos_R) (*arr_pf_right_itteration[mode])(); /* ���������� �������������� ������ `mode` ������� */
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
//�������  ����������                      // 0 0 0 
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
//������� ����� � �����������              // 1 0 0
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
//  ������ ������� �����                   // 1 1 1
void left_itr_7(void)
{
	int i=0;
	unsigned char nibble_l=0, nibble_h=0;
		for(i=0;i<4;i++)
		{
			nibble_l=~(1<<arrL[i]);  // 11111110  // --������� ����
			nibble_l&=0b00011011;    // 00011010 // ���������� ��� ���� ����� ���������
			nibble_h=PORTD&11100100; // ***00*00 // ����� ��������� ��������� ����
			PORTD=nibble_h|nibble_l; // ***11 10 //
			pause_t1_ms(100);        // ����� ������� ����� LEDs
		}
		//PORTD&=~((1<<0)|(1<<1)|(1<<3)|(1<<4)); // ON
		//pause_t1_ms(100);
		PORTD|=(1<<0)|(1<<1)|(1<<3)|(1<<4); //OFF
		pause_t1_ms(400); // ���������� ��������. ����� ����� �����������.
}
void right_itr_7(void)
{
	PORTB&=~(1<<7);pause_t1_ms(100);        
	PORTB|=(1<<7);PORTB&=~(1<<6);pause_t1_ms(100);
	PORTB|=(1<<6);PORTB&=~(1<<5);pause_t1_ms(100);
	PORTB|=(1<<5);PORTD&=~(1<<6);pause_t1_ms(100);
	PORTD|=(1<<6);pause_t1_ms(400); // ���������� ��������. ����� ����� �����������.	
}