/*
 * Timers.c
 *
 *  Created on: 28 oct 2021
 *      Author: sergio_mndz
 */

#include "Timers.h"

osaEventId_t timerEvents;
/* Global Variable to store our TimerID */
tmrTimerID_t timer1sID;
/* Handler ID for task */
osaTaskId_t gTimerTaskHandler_ID;
/* Local variable to store the current state of the LEDs */
//static ledStatus_t ledsState = RED;
uint8_t gCount = 1;

void increment_counter(void);

/* This is the function called by the Timer each time it expires */
static void taskTimerCallback(void *param)
{
	OSA_EventSet(timerEvents, gTimerTaskEvent1_c);

	/**este evento sera usado para incrementar el contador*/
}

void My_Task(osaTaskParam_t argument)
{
	osaEventFlags_t customEvent;
	while(1)
	{
		OSA_EventWait(timerEvents, osaEventFlagsAll_c, FALSE, osaWaitForever_c,
				&customEvent);
		if( !gUseRtos_c && !customEvent)
		{
			break;
		}
		/* Depending on the received event */
		switch(customEvent){
		case gTimerTaskEvent1_c:
			/* 1 seconds passed so increment count */
			increment_counter();
			break;
		default:
			break;
		}

	}
}

/* OSA Task Definition*/
OSA_TASK_DEFINE(My_Task, gTimerTaskPriority_c, 1, gTimerTaskStackSize_c, FALSE );

/* Function to init the task */
void timerTask_Init(void)
{
	timerEvents = OSA_EventCreate(TRUE);
	/* The instance of the MAC is passed at task creaton */
	gTimerTaskHandler_ID = OSA_TaskCreate(OSA_TASK(My_Task), NULL);
}

void timer1s_Start(void)
{
	timer1sID = TMR_AllocateTimer();
	TMR_StartIntervalTimer(timer1sID, /*myTimerID*/
			1000, /* Timer's Timeout */
			taskTimerCallback, /* pointer to myTaskTimerCallback function */
			NULL
	);
}

void increment_counter(void)
{
	gCount++;
	if(gCount > 200)
		gCount = 0;
}

uint8_t getCounter(void)
{
	return gCount;
}
