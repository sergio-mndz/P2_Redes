/*
 * LED_Control.c
 *
 *  Created on: 2 nov 2021
 *      Author: sergio_mndz
 */

#include "LED_Control.h"

void turn_LED(uint8_t counter)
{
	LED_TurnOffAllLeds();

	switch (  counter )
	{
	case  0 :
		LED_TurnOnLed(LED2);
		break;
	case  1  :
		LED_TurnOnLed(LED3);
		break;
	case  2 :
		LED_TurnOnLed(LED4);
		break;
	case  3 :
		LED_TurnOnAllLeds();
		break;
	}
}

void toggle_w_LED()
{
	//LED_ToggleLed(LED2);

	static uint8_t init = 0;

	if( 0== init)
	{
		LED_Init();
	}

	static uint8_t flag_toggle = 0 ;

	if ( 0 == flag_toggle)
	{
		LED_TurnOnAllLeds();
		flag_toggle = 1 ;
	}
	else
	{
		LED_TurnOffAllLeds();
		flag_toggle = 0 ;
	}


}
