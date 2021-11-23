/*
 * Timers.c
 *
 *  Created on: 28 oct 2021
 *      Author: sergio_mndz
 */

#include "Timers.h"

osaEventId_t timerEvents;
/* Global Variable to store our TimerID */
tmrTimerID_t timer5sID;
tmrTimerID_t timer7sID;
/* Handler ID for task */
osaTaskId_t gTimerTaskHandler_ID;
/* Local variable to store the current state of the LEDs */
//static ledStatus_t ledsState = RED;

/* This is the function called by the Timer each time it expires */
static void taskTimerCallback(void *param)
{
	//OSA_EventSet(timerEvents, gTimerTaskEvent1_c);
	timer5s_extern_callback();
}

static void taskTimer7sCallback(void *param)
{
	//OSA_EventSet(timerEvents, gTimerTaskEvent1_c);
	timer7s_extern_callback();
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
			// TODO: Se puede hacer el request desde aquí?
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

void timer5s_Start(void)
{
	timer5sID = TMR_AllocateTimer();
	TMR_StartIntervalTimer(timer5sID, /*myTimerID*/
			5000, /* Timer's Timeout */
			taskTimerCallback, /* pointer to myTaskTimerCallback function */
			NULL
	);
}



void timer7s_Start(void)
{
	timer7sID = TMR_AllocateTimer();
	TMR_StartIntervalTimer(timer7sID, /*myTimerID*/
			7000, /* Timer's Timeout */
			taskTimer7sCallback, /* pointer to myTaskTimerCallback function */
			NULL
	);

}
