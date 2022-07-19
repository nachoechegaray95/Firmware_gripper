#include "sapi.h"      // <= sAPI header
#include "sapi_i2c.h"
#include "../inc/ina219.h"
#include "../inc/num.h"

#define SERVO_GRIPPER   SERVO8

int obtener_num_ascii(int dato);
void sumar_numeros_angulo(int *servo_angulo, int angulo_ingresado, int ponderacion);


int main(void)
{

   boardConfig();
   uartConfig( UART_USB, 115200 );

   uint8_t address = 0x40;
   bool_t valor = 0;

   //printf("Inicializando INA219...\r\n" );
   if(!ina219Init(address)){
      printf( "INA219 no inicializado\r\n" );
      while(!ina219Init(address));
   }
   printf("INA219 inicializado correctamente.\r\n\r\n" );

   valor = servoConfig( 0, SERVO_ENABLE );
   valor = servoConfig( SERVO_GRIPPER, SERVO_ENABLE_OUTPUT );


   uint8_t dato = 0;
   int angulo_ingresado = 0, servo_angulo = 0;

   int ponderacion = 0;
   bool_t angulo_valido = TRUE;
   while(TRUE) {

	      while(uartReadByte( UART_USB, &dato ))
	      {
	    		angulo_ingresado = obtener_num_ascii(dato);
	    		sumar_numeros_angulo(&servo_angulo, angulo_ingresado, ponderacion);
	    		ponderacion++;
	    		delay (DELAY_ENTRE_LECTURAS);
	    	}
	      if (ponderacion > 0)
	      {
    		  ponderacion = 0;

	    	  if ((servo_angulo < 0 || servo_angulo >180) )
	    	  {
	    		if (angulo_valido == TRUE){
	    		  angulo_valido = FALSE;
	    		  printf("Ángulo inválido \r\n");
	    		}
	    	  }
	    	  else
	    	  {
	    		  angulo_valido = TRUE;
	    		  ponderacion = 0;
	    		  gpioWrite( LEDR, 1 );
	    		  servoWrite( SERVO_GRIPPER, servo_angulo);
	    		  ina219Read();
	    		  ina219ObtenerCorriente();
	    		  delay(DELAY_ENTRE_LECTURAS);
	    		  gpioWrite( LEDR, 0 );
	    	  }

	      }


		  if (!gpioRead( TEC1 ))
		  {
			  gpioWrite( LEDB, 1 );
			  if (servo_angulo >= ANGULO_MAX - INCREMENTO_ANGULO_PASO)
				  servo_angulo = ANGULO_MAX;
			  else
				  servo_angulo += INCREMENTO_ANGULO_PASO;

			  servoWrite( SERVO_GRIPPER, servo_angulo);
			  ina219Read();
			  ina219ObtenerCorriente();
			  delay(200);
			  gpioWrite( LEDB, 0 );

		  }

		  if (!gpioRead( TEC2 ))
		  {
			  gpioWrite( LED1, 1 );
			  if (servo_angulo <= ANGULO_MIN + DECREMENTO_ANGULO_PASO)
				  servo_angulo = ANGULO_MIN;
			  else
				  servo_angulo -= DECREMENTO_ANGULO_PASO;

			  servoWrite( SERVO_GRIPPER, servo_angulo);
			  ina219Read();
			  ina219ObtenerCorriente();

			  delay(DELAY_ENTRE_LECTURAS);
			  gpioWrite( LED1, 0 );


		  }

		  if (!gpioRead( TEC3 ))
		  {
			  gpioWrite( LED2, 1 );
			  while(servo_angulo > ANGULO_MIN)
			  {
				  servo_angulo -= DECREMENTO_ANGULO_VMIN;
				  delay(DELAY_VMIN);
				  servoWrite( SERVO_GRIPPER, servo_angulo);
				  ina219Read();
				  ina219ObtenerCorriente();
			  }
			  gpioWrite( LED2, 0 );

		  }

		  if (!gpioRead( TEC4 ))
		  {
			  gpioWrite( LED3, 1 );
			  while(servo_angulo < ANGULO_MAX)
			  {
				  servo_angulo += INCREMENTO_ANGULO_VMIN;
				  delay(DELAY_VMIN);
				  servoWrite( SERVO_GRIPPER, servo_angulo);
				  ina219Read();
				  ina219ObtenerCorriente();
			  }

			  gpioWrite( LED3, 0 );

		  }
   }
   return 0 ;
}


