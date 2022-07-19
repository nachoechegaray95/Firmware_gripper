#include "../inc/num.h"

int obtener_num_ascii(int dato)
{
	int numero = 0;
	   switch(dato) {
	      case 48:
	         numero = 0;
	         break;
	      case 49:
	         numero = 1;
	         break;
	      case 50:
	         numero = 2;
	         break;
	      case 51:
	         numero = 3;
	         break;
	      case 52:
	         numero = 4;
	         break;
	      case 53:
	         numero = 5;
	         break;
	      case 54:
	         numero = 6;
	         break;
	      case 55:
	         numero = 7;
	         break;
	      case 56:
	         numero = 8;
	         break;
	      case 57:
	         numero = 9;
	         break;

	      default:
	         break;
	   }

	return numero;
}

void sumar_numeros_angulo(int *servo_angulo, int angulo_ingresado, int ponderacion)
{
	if(ponderacion == 0)
		*servo_angulo = angulo_ingresado;

	if(ponderacion == 1)
		*servo_angulo = *servo_angulo*10 + angulo_ingresado;

	if(ponderacion == 2)
		*servo_angulo = *servo_angulo*10 + angulo_ingresado;

}
