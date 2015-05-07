#include <18F4550.h>

#use delay(clock=12000000)
#fuses HS,NOWDT,PUT,NOLVP 

boolean time3 = 0;
boolean led = 0;
#int_timer1
void timer1(void)
{   
   static int contador;
   set_timer1(1673 + get_timer1());
   contador++;
   
   if(contador == 375) 
   {
      
      led = !led;
      
      contador = 0;
      
    }  
}

   void main()
{
   
   
   setup_timer_1 (RTCC_INTERNAL | RTCC_DIV_256);
   set_timer1 (1673); // inicia o timer 1 em 1673
   // habilita interrupções
   enable_interrupts (global | int_timer1);
   while (true){
      if(led==1){
    output_high(PIN_A0);
      }
      
      else{
    output_low(PIN_A0);
    }
      }

}
