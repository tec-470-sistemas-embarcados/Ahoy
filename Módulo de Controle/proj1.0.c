#include <htc.h>
#include <stdio.h>
#include "usart.h"

// Frequência de Clock do Micro
#ifndef _XTAL_FREQ
	#define _XTAL_FREQ 20000000
#endif

#define porta_in RB4
#define cortina_in RB5
#define aquecimento_in RB6
#define escotilha_in RB7

#define LED_LUZ RB0
#define LED_TEMP RB1
#define porta_out RB2
#define cortina_out RB3
#define aquecimento_out RA4
#define escotilha_out RA5
#define bateria_in RA2
#define RX RC7
#define TX RC6


unsigned int DATA = 0; 
unsigned int dataTemp = 0;
unsigned int dataLuz = 0;
unsigned int dataBat = 0;
unsigned int teste_escot = 0;


int check= 0;
bit flag = 0;
bit flag1 = 0; 

int portaFlag = 0;

bit ant4 = 0;
bit ant5 = 0;
bit ant6 = 0;
bit ant7 = 0;

unsigned char count = 0;
unsigned char portaCount = 0;
float temp = 0;
float luz = 0;
float bateria = 0;
// A função da interrupção deve ser void e interrupt e com qualquer nome

void interrupt interupt(void)
{	

/*	if(INTF) //Interrupção do botão
	{
		flag1 = ~flag1;
	
		 	INTF = 0;
	}*/


		
	if(RBIF){
	
		if(ant4!=porta_in)
		{
			if(porta_in==1)
			{
				ant4 = 1;
				portaFlag = 0; //fechou a porta, para de contar os 30s
			}else
			{
	
				ant4 = 0;
				cortina_out = 1;
			}

			porta_out = ant4;
		}
	if(ant5!=cortina_in)
		{
			if(cortina_in==1)
			{
				ant5= 1;
			}else
			{
		
				ant5 = 0;
			}
			cortina_out = ant5;
		}

	if(ant6!=aquecimento_in)
		{
			if(aquecimento_in==1)
			{
				ant6= 1;
			}else
			{
		
				ant6 = 0;
			}
			aquecimento_out = ant6;
		}
	if(ant7!=escotilha_in)
		{
			if(escotilha_in==1)
			{
				ant7= 1;
			}else
			{
		
				ant7 = 0;
			}
			escotilha_out = ant7;
		}

		__delay_ms(300);
	

	
	

        	RBIF = 0; //limpa o flag
		}


	if(TMR0IF) // Contador
	{
		portaCount++;
		count ++;
		if(count == 153)
		{			
			CHS0 = 0;	
			CHS1 = 0;
			CHS2 = 0;
				__delay_ms(1);
			GO_DONE = 1; 

			flag = ~flag;

			count = 0;
			check=1;
		}

		if(count == 75)
		{		
			CHS0 = 1;	
			CHS1 = 0;
			CHS2 = 0;
				__delay_ms(1);
			GO_DONE = 1; 
			check = 0;	
		}

		if(count == 231)
		{		
			CHS0 = 0;	
			CHS1 = 1;
			CHS2 = 0;
				__delay_ms(1);
			GO_DONE = 1; 
			check = 2;	
		}
		//porta aberta por mais de 30s
	/*	if(portaCount >= 30000 && portaFlag==1)
		{
			cortina_out = 1;
		}*/
		
		TMR0IF = 0;
	}
	
	if(ADIF) // interrupção modulo ad
	{
//
		DATA = (ADRESH <<2) + (ADRESL >>6);

		if(check==1){ dataTemp = DATA; }
		if(check==0){ dataLuz = DATA; }
		if(check==2){ dataBat = DATA; }

		
		ADIF = 0;
	}

}



// Interrupção Externa RB0
// Interrupção Interna

void main(void)
{
	__CONFIG(CP_OFF & WDTE_OFF & BOREN_OFF & PWRTE_OFF & FOSC_HS & WRT_OFF & LVP_OFF & CPD_OFF);	

	TRISA0 = 1;
	TRISA1 = 1;
	TRISA5 = 1;
	TRISA4 = 0;
	TRISA5 = 0;
	

	TRISB = 0b11111111; // CONFIGURA TODAS AS PORTAS B EM ORDEM
	TRISB0 = 0;
	TRISB1 = 0; //CONFIGURA DIZENDO QUE ESSE PINO SERA USADO PRA SAIDA (0) ENTRADA SERIA (1)
	TRISB2 = 0;
	TRISB3 = 0;
	TRISB6 = 1;

//botão	----------------------
	INTEDG = 1;
	INTF = 0;		// O flag de interrupção externa que é alterada pelo circuito
	INTE = 1;		// Flag de interrupção externa deverá ser setado em 1 para poder ocorrer a interrupção
//----------------------------


	T0CS = 0; //Timer no modo TIMER
	T0SE = 0;
	PSA = 0;
	PS2 = 1;
	PS1 = 1;	
	PS0 = 1;

	TMR0IF = 0;
	TMR0IE = 1;
	RBIF = 0; 		//flag
	RBIE = 1;		//habilita as interrupçoes da porta change
	PEIE = 1;
	GIE = 1;
	

	init_comms(); // set up the USART - settings defined in usart.h
	
	//configura modulo ac


	//CONFIGURAÇÃO DO MODULO AD
	ADCS1 = 1;	// FOSC/64
	ADCS2 = 1;
	ADCS0 = 0;

	CHS0 = 0;	// CANAL ZERO
	CHS1 = 0;
	CHS2 = 0;
	
	ADFM = 0; //JUSTIFICATIVA A ESQUERDA

	PCFG3 = 0; //CONFIGURAÇÃO DE FUNÇÃO DOS PINOS
	PCFG2 = 1;
	PCFG1 = 0;
	PCFG0 = 0;

	ADIF = 0;
	ADIE = 1;


	


	/* Teste de Inicialização
	LED_01 = 1;
	__delay_ms(500);
	LED_01 = 0;
	__delay_ms(500);
	LED_01 = 1;
	__delay_ms(500);
	LED_01 = 0;
	__delay_ms(500);
	*/

	ADON = 1; //LIGA O MODULO AD
	
	GO_DONE = 1;	

porta_out = 0;
cortina_out=0;
aquecimento_out = 0;
escotilha_out = 0;

	while(1)
	{

	//temperatura
	//	printf("\nA DATA eh: %d", dataTemp);  // LM35 - -55º C ~~ 150º

		temp = (5 * dataTemp / 1023.0)*100;

	//	printf("\nA temperatura eh: %f", temp);
	
		//temperatura
		if(temp > 20)
		{
			LED_TEMP = 1;	
		}else
		{
			LED_TEMP = 0;
		}
       
		
		//====================================

		//Luz

		luz = (5 * dataLuz / 1023.0);

		printf("\nA luminosidade eh: %f", luz);

		if(luz < 1.20) //nivel 4 de luminosidade, mais escuro
		{
			LED_LUZ = 1;
		}
		else if ( luz >= 1.20 && luz < 2.4)
		{
			LED_LUZ = 1;
		}
		else if ( luz >= 2.4 && luz < 4.8)
		{
			LED_LUZ = 0;
		}
		else if ( luz >= 4.8)//mais claro
		{
			LED_LUZ = 0;
		}

	//Bateria------------------------------------

	//ValorReal = Entrada / 2^10 - 1
	bateria = (5 * dataBat / 1023.0);
	
	if(bateria>=2.45) printf("\nA bateria esta acima da metade");
	else printf("\nA bateria esta abaixo da metade");

	//------------------------------------------------
	
	

	//logica do sistema
		//desliga o sistema de aquecimento
		if(temp > 40)
		{
			aquecimento_out = 0;	
		}

		//sistema de aquecimento desligado = escotilha abre ou fecha (muda de estado)
		if(aquecimento_out == 0 && teste_escot==0)
		{
			//escotilha_out = ~escotilha_out;
			if(escotilha_out==0)escotilha_out=1;
			else escotilha_out=0;
			teste_escot=1;
		}
		if(aquecimento_out == 1 && teste_escot==1)
		{
				teste_escot=0;
		}
		
		//se a porta foi aberta 30s
		if(porta_in == 0)
		{
			portaFlag = 1;
			portaCount = 0;
			
		}
		
} 	
}
