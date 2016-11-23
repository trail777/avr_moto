/*
 * moto1_Attiny_13A.c
 *
 * Created: 08.11.2016 
 *  Author: Gleb
/////////////////////////////////////////////////////////
********* ATtiny 13a 
 Fuses:
Lock Bits: mode 1
SPIEN=0;                                                        //  SPI programming enable
BODLEVEL=00;  // Vcc=4.3�                                       //  Enable Bodlevel_1,Bodlevel_0
Int RC Osc. 9.6MHz; Start-up time: 14CK+65ms(CKSEL=11 SUT=10)   //  Select SUT0, CKSEL0
CKDIV8

 ////////////////// PORTB /////////////////
 ****  PORT PB0 - �� ���������. ���������� ��� ���� � ����������  ���������
 ****  PORT PB1 --�������������� ����-�������               (���� � ������� ���������)  �������������� ������������ ��������� ������ �������				
 ****  PORT PB2 --������������� ���.4                       (���� � ������� ���������)
 ****  PORT PB3 --���� ������������ �� ������� ATTiny2313a �� ������������  /// ����� ���� � �����. ���������
 ****  PORT PB4 --����-������                               (����� - "1")
 ****  PORT PB5 --RESET   (������� ��������)                /// default
 
 //////////////////////////////////////////
 //   Switch     4         mode   
 //             PB4    
 //              0          0
 //              1          1
 // **************************************
 */
#ifndef F_CPU
#define F_CPU 1200000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
 
volatile unsigned int num_timer0;
volatile unsigned char flag_stop_on;
unsigned char mode;

ISR (PCINT0_vect);         // ����� ������� � ������� Interrupt Vectors 3 
ISR (TIM0_COMPA_vect);     // ����� ������� � ������� Interrupt Vectors 7 

void pause_t0_ms(unsigned int interval_ms);	
void stop_0(void), stop_1(void);

void (*arr_pf_stop_itteration []) (void) = { //������������� ������� ���������� �� ������� stop-�������
	stop_0,   // ������� stop-������           // 0 
	stop_1,   // stop-������ ������������      // 1
};
void Handler_stop_detection(unsigned char); 
unsigned char read_mode(void);

int main(void)
{
    //// ��������� �������������� ���������
	PRR|=(1<<PRADC); // Disabled ADC
	
	////INIT ports //////////////////////////
    DDRB=0b10000;
    PORTB=0b11001;
	asm volatile( "nop \n" "nop \n" "nop \n");
	//////////////////////////////////////////////////////////////
	
	/// INIT ext interrupts  INT0
    // MCUCR&=~(1<<ISC01); MCUCR|=(1<<ISC00);    // ����������� ���������� INT0 ������ ������ �������
	// GIMSK|=(1<<INT0);                         // Enabled interrupt INT0 
	//////////////////////////////////////////////////////////////
    
    /// INIT ext interrupts  PCINT1
    PCMSK|=(1<<PCINT1);
    GIMSK|=(1<<PCIE);
    //////////////////////////////////////////////////////////////
	
	/// INIT SLEEP MODE ///////////////////
	MCUCR&=~(1<<SM0); MCUCR|=(1<<SM1);    // Power-down
	MCUCR|=(1<<SE);                       // Sleep_Enable 
	////////////////////////////////////////////////////////////
	
	//////////////////////// init timers0 (8 bit) ////////////////
	OCR0A=0x96;          // � ������� ��������� ������� 1�� -->1200000\8*0,001=150; (0x96)
    sei();
	TIMSK0|=1<<OCIE0A;   // ��������� ���������� �� ����������	
	asm volatile( "nop \n" "nop \n" "nop \n");
	///////////////////////////////////////////////////////////////
	mode=read_mode(); //���������� ����� (������������� (0) ��� �������(1))
    /// ��������� ��������� ������ ������� �� (�������/������� ������) ����� ��������� ��
    {
        unsigned char i=0,count_S1=0;
        while (i<160)
        {
            if (!(PINB&(1<<1))) count_S1++;
            i++;
        }
        if (count_S1>100)  flag_stop_on=1;
        else flag_stop_on=0;
    }   
	for (;;)
	{
       
       Handler_stop_detection(mode);            // ������ ��������������� ������� � ����� ��� ������� �� ������ �������
       PORTB|=(1<<4);                           //��������� ����/������ ��� �������
       asm volatile( "nop \n" "nop \n" "nop \n");
       __asm__ __volatile__ ( "sleep" "\n\t"::); //��������, ���� �� ���� �� ������ �������
       
      // � ����� ����������� ����� �������� ��� ������� �� ������
     
	} 
}

ISR (TIM0_COMPA_vect)  {TCNT0=0; num_timer0++;}

ISR (PCINT0_vect) // ������ �� ������ ������� ��� ���������
{
    unsigned char i=0,count_S1=0;
    while (i<160)
      {
          if (!(PINB&(1<<1))) count_S1++;   
          i++;
      }
      if (count_S1>100)  flag_stop_on=1;
      else flag_stop_on=0;          
}


void pause_t0_ms(unsigned int interval_ms)
{
	TCNT0=0;
	num_timer0=0;
	TCCR0B|=(1<<CS01); //Prescaler=8-->F_CPU/8-->������ ��������  
	while(1) { if(num_timer0==interval_ms) {TCCR0B&=~(1<<CS00|1<<CS01|1<<CS02);break;}}
}


void Handler_stop_detection(unsigned char mode) 
{
	asm volatile( "nop \n" "nop \n" "nop \n" "nop \n" "nop \n" "nop \n" "nop \n" "nop \n "); //
    while (flag_stop_on) (*arr_pf_stop_itteration[mode])();  //��������� �-� ��� ������� �� ������ �������
}

unsigned char read_mode(void)
{
	unsigned char result=0; 
    asm volatile( "nop \n" "nop \n"); //
     if (!(PINB&(1<<2))) result=1; //mode 1 ������� ����� �������������
     else result=0;                //mode 0 ������� ����� (���/����) 
	return result;
}
////////////////////////////////////////////////////////////////////////
//�������  ����-������                      
void stop_0 (void)
{
	PORTB&=~(1<<4);    // ON    
}
//������������ ����-������
void stop_1 (void)
{
    PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
    PORTB&=~(1<<4); pause_t0_ms(120);PORTB|=(1<<4);pause_t0_ms(120);
    PORTB&=~(1<<4); pause_t0_ms(30);PORTB|=(1<<4);pause_t0_ms(40);
    PORTB&=~(1<<4); pause_t0_ms(50);PORTB|=(1<<4);pause_t0_ms(60);
    PORTB&=~(1<<4); pause_t0_ms(60);PORTB|=(1<<4);pause_t0_ms(80);
    if (flag_stop_on==0) return;
	PORTB&=~(1<<4); pause_t0_ms(55);PORTB|=(1<<4);pause_t0_ms(80);
	PORTB&=~(1<<4); pause_t0_ms(60);PORTB|=(1<<4);pause_t0_ms(80);
	PORTB&=~(1<<4); pause_t0_ms(60);PORTB|=(1<<4);pause_t0_ms(80);
	PORTB&=~(1<<4); pause_t0_ms(60);PORTB|=(1<<4);pause_t0_ms(80);
	PORTB&=~(1<<4); pause_t0_ms(60);PORTB|=(1<<4);pause_t0_ms(80);
	if (flag_stop_on==0) return;
    
	PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
	PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
    if (flag_stop_on==0) return;
	PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
	PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
    if (flag_stop_on==0) return;
    PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
    PORTB&=~(1<<4); pause_t0_ms(200);PORTB|=(1<<4);pause_t0_ms(200);
    
	if (flag_stop_on==0) return;
	PORTB&=~(1<<4); pause_t0_ms(300);PORTB|=(1<<4);pause_t0_ms(300);
    if (flag_stop_on==0) return;
	PORTB&=~(1<<4); pause_t0_ms(350);PORTB|=(1<<4);pause_t0_ms(350);
    if (flag_stop_on==0) return;
	PORTB&=~(1<<4); pause_t0_ms(400);PORTB|=(1<<4);pause_t0_ms(400);
    if (flag_stop_on==0) return;
    PORTB&=~(1<<4); pause_t0_ms(500);PORTB|=(1<<4);pause_t0_ms(500);
    if (flag_stop_on==0) return;
	//PORTB&=~(1<<4); pause_t0_ms(1500);PORTB|=(1<<4);
    PORTB&=~(1<<4); while(flag_stop_on); PORTB|=(1<<4);pause_t0_ms(500); // �������� ����-������ ��� ������ �� ������� ������
}
