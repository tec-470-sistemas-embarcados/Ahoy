//===================================================================
// INCLUDES
//-------------------------------------------------------------------
#include<htc.h>

//===================================================================
// CONFIGURATION
//-------------------------------------------------------------------

#ifndef _XTAL_FREQ
	#define _XTAL_FREQ 20000000
#endif

//===================================================================
// VARIABLES
//-------------------------------------------------------------------
unsigned char reading;
unsigned char buffer[64];
//Comandos AT envio sms
unsigned char comunication[] = "AT\r";
unsigned char sms_format_text[]="AT+CMGF=1\r";   // TEXT mode
unsigned char character_set_gsm[]="AT+CSCS=\"GSM\"\r";
unsigned char sms_nunber[]="AT+CMGS=\"XXXXXXXXXX\"\r";
unsigned char sms_msg[] = "";
unsigned char sms_read[] = "AT+CMGR=1\r";
//unsigned int qtd_msg = 0;

//===================================================================
// PRIVATE PROTOTYPES
//-------------------------------------------------------------------
char UART_Init(const long int baundrate);
void UART_Write(char data);
char UART_TX_Empty();
void UART_Write_Text(char *text);
char UART_Data_Ready();
char UART_Read();
void UART_Read_Text(char *Output, unsigned int length);
void gprs_interrupt();

//===================================================================
// INTERRUPT HANDLER VECTORS
//-------------------------------------------------------------------
void interrupt() //  ISR
{
 if(UART_Data_Ready())
	{
		gprs_interrupt();
	}
}

void main(void)
{
	INTCON.GIE = 1; // Enable The Global Interrupt
  	INTCON.INTE = 1; // Enable INT
	UART_Init(9600);
	
	UART_Write_Text(comunication);
	delay_ms(500);
	UART_Write_Text(sms_format_text);
	delay_ms(500);
	UART_Write_Text(character_set_gsm);
	delay_ms(500);

	while(1){

	}
}

/******************************************************************************
 * Function:        gprs_interrupt
 * Note:            
 *
 *****************************************************************************/
void gprs_interrupt()
{
	UART_Write_Text(sms_read);
	delay_ms(500);

	UART_Read_Text(buffer, 64);
	delay_ms(500);
	//Tem que saber o tamanho da mensagem real para saber a posição 
	//no vetor da informação que a gente quer. Em seguida é só enviar.
	//Falta colocar o numero que vai receber a mensagem
	UART_Write_Text(sms_nunber);
	delay_ms(500);
	UART_Write_Text("Envia a informação que quiser");
	delay_ms(500);

}

/******************************************************************************
 * Function: UART_Init
 * Note: Inicializa a UART com o baundrate especificiado do dispositivo            
 *
 *****************************************************************************/
void UART_Init(const long int baundrate)
{
  unsigned int x;
  x = (_XTAL_FREQ - baudrate*64)/(baudrate*64); //SPBRG for Low Baud Rate
  if(x>255) //If High Baud Rate required
  {
    x = (_XTAL_FREQ - baudrate*16)/(baudrate*16); //SPBRG for High Baud Rate
    BRGH = 1; //Setting High Baud Rate
  }
  if(x<256)
  {
    SPBRG = x; //Writing SPBRG register
    SYNC = 0; //Selecting Asynchronous Mode
    SPEN = 1; //Enables Serial Port
    TRISC7 = 1;  
    TRISC6 = 1; 
    CREN = 1; //Enables Continuous Reception
    TXEN = 1; //Enables Transmission
    
  }
   
}

/******************************************************************************
 * Function:        UART_TX_Empty
 * Note:            
 *
 *****************************************************************************/
char UART_TX_Empty()
{
  return TRMT; //Returns Transmit Shift Status bit
}



/******************************************************************************
 * Function:        UART_Write_Text
 * Note:            
 *
 *****************************************************************************/
void UART_Write_Text(char *text)
{
  int i;
  for(i=0;text[i]!='\\0';i++)
    	UART_Write(text[i]);
}

/******************************************************************************
 * Function:        UART_Data_Ready
 * Note:            
 *
 *****************************************************************************/
char UART_Data_Ready()
{
  return RCIF;
}

/******************************************************************************
 * Function:        UART_Read
 * Note:            
 *
 *****************************************************************************/
char UART_Read()
{
  while(!RCIF); //Waits for Reception to complete
  return RCREG; //Returns the 8 bit data
}


/******************************************************************************
 * Function:        UART_Read_Text
 * Note:            
 *
 *****************************************************************************/
void UART_Read_Text(char *Output, unsigned int length)
{
  int i;
  for(int i=0;i<length;i++)
    Output[i] = UART_Read();
}
