#include <18F4550.h>

#use delay(clock=48000000)
#fuses HS,NOWDT,PUT,NOLVP 


#int_timer1
void timer1(void)
{   
  
  static boolean led = 0;
   static int contador;
   set_timer1(65161 + get_timer1()); //O timer deve contar apenas 375 entao deve começar de 65161
   contador++;
   
   if(contador == 375)  // 375 equivale a 3 segundos, já que 1 segundo é 125
   {
      
      led = !led;        //LED piscando 
      
      contador = 0;    // Zerando o contador
      
    }  
}

   void main()
{
   
   
   setup_timer_1 (RTCC_INTERNAL | RTCC_DIV_256);
   set_timer1 (65161); // inicia o timer 1 em 65161
   // habilita interrupções
   enable_interrupts (global | int_timer1);
   while (true){
      if(led==1){
    output_high(PIN_A0);  // Ligando o LED
      }
      
      else{
    output_low(PIN_A0);
    }
      }

}
