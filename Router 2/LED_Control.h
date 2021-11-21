/*
 * LED_Control.h
 *
 *  Created on: 2 nov 2021
 *      Author: sergio_mndz
 */

#ifndef LED_CONTROL_H_
#define LED_CONTROL_H_

/* Fwk */
#include "TimersManager.h"
#include "FunctionLib.h"
#include "LED.h"
/* KSDK */
#include "fsl_common.h"
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"
#include "Keyboard.h"

void turn_LED(uint8_t counter);

void toggle_w_LED();

#endif /* LED_CONTROL_H_ */
