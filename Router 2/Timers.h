/*
 * Timers.h
 *
 *  Created on: 28 oct 2021
 *      Author: sergio_mndz
 */

#ifndef TIMERS_H_
#define TIMERS_H_

/* Fwk */
#include "TimersManager.h"
#include "FunctionLib.h"
/* KSDK */
#include "fsl_common.h"
#include "EmbeddedTypes.h"
#include "fsl_os_abstraction.h"

/* Define the available Task's Events */
#define gTimerTaskEvent1_c (1 << 0)
#define gTimerTaskEvent2_c (1 << 1)
#define gTimerTaskEvent3_c (1 << 2)

#define gTimerTaskPriority_c 3
#define gTimerTaskStackSize_c 400

void timer5s_Start(void);

void timerTask_Init(void);

#endif /* TIMERS_H_ */
