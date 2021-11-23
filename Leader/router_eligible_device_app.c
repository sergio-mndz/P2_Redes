/*
 * Copyright (c) 2014 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!=================================================================================================
\file       router_eligible_device_app.c
\brief      This is a public source file for the router eligible device demo application.
==================================================================================================*/

/*==================================================================================================
Include Files
==================================================================================================*/
/* General Includes */
#include "EmbeddedTypes.h"
#include <string.h>

/* FSL Framework */
#include "shell.h"
#include "Keyboard.h"
#include "RNG_Interface.h"
#include "TimersManager.h"

// Parte 2
#include "board.h"
#include "math.h"
#include "fsl_fxos.h"
#include "fsl_i2c.h"
#include "fsl_tpm.h"
#include "fsl_port.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

#include "Timers.h"

/* Network */
#include "ip_if_management.h"
#include "event_manager.h"

/* Application */
#include "router_eligible_device_app.h"
#include "shell_ip.h"
#include "thread_utils.h"
#include "thread_meshcop.h"
#include "thread_network.h"
#include "thread_app_callbacks.h"
#include "thread_attributes.h"

#include "app_init.h"
#include "app_stack_config.h"
#include "app_thread_config.h"
#include "app_led.h"
#include "app_temp_sensor.h"
#include "coap.h"
#include "app_socket_utils.h"
#if THR_ENABLE_EVENT_MONITORING
#include "app_event_monitoring.h"
#endif
#if THR_ENABLE_MGMT_DIAGNOSTICS
#include "thread_mgmt.h"
#include "thci.h"
#endif
#if UDP_ECHO_PROTOCOL
#include "app_echo_udp.h"
#endif

/*==================================================================================================
Private macros
==================================================================================================*/
#ifndef APP_MSG_QUEUE_SIZE
#define APP_MSG_QUEUE_SIZE                  20
#endif

#if (THREAD_USE_SHELL == FALSE)
#define shell_write(a)
#define shell_refresh()
#define shell_printf(a,...)
#endif

#define gThrDefaultInstanceId_c                 0
#if APP_AUTOSTART
#define gAppFactoryResetTimeoutMin_c            10000
#define gAppFactoryResetTimeoutMax_c            20000
#endif
#define gAppRestoreLeaderLedTimeout_c           60     /* seconds */

#define gAppJoinTimeout_c                       800    /* miliseconds */

#define APP_LED_URI_PATH                        "/led"
#define APP_TEMP_URI_PATH                       "/temp"
#define APP_SINK_URI_PATH                       "/sink"

//LAB
#define APP_RESOURCE1_URI_PATH					"/resource1"
#define APP_RESOURCE2_URI_PATH					"/resource2"
//practica
#define APP_TEAM9_URI_PATH						"/team9"
#define APP_ACCEL_URI_PATH                      "/accel"

#if LARGE_NETWORK
#define APP_RESET_TO_FACTORY_URI_PATH           "/reset"
#endif

#define APP_DEFAULT_DEST_ADDR                   in6addr_realmlocal_allthreadnodes

// Practica parte 2
/* The TPM instance/channel used for board */
#define BOARD_TIMER_BASEADDR TPM2
#define BOARD_FIRST_TIMER_CHANNEL 0U
#define BOARD_SECOND_TIMER_CHANNEL 1U
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_Osc0ErClk)
#define TIMER_CLOCK_MODE 1U
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C1_CLK_SRC
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTC
#define I2C_RELEASE_SCL_PORT PORTC
#define I2C_RELEASE_SDA_GPIO GPIOC
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOC
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U
/* Upper bound and lower bound angle values */
#define ANGLE_UPPER_BOUND 85U
#define ANGLE_LOWER_BOUND 5U

/*==================================================================================================
Private type definitions
==================================================================================================*/

/*==================================================================================================
Private global variables declarations
==================================================================================================*/
static instanceId_t mThrInstanceId = gInvalidInstanceId_c;    /*!< Thread Instance ID */

static bool_t mFirstPushButtonPressed = FALSE;

static bool_t mJoiningIsAppInitiated = FALSE;

/*==================================================================================================
Private prototypes
==================================================================================================*/
static void App_HandleKeyboard(uint8_t *param);
static void App_UpdateStateLeds(appDeviceState_t deviceState);
static void APP_JoinEventsHandler(thrEvCode_t evCode);
static void APP_InitCoapDemo(void);
static void APP_ReportTemp(uint8_t *pParam);
static void APP_SendDataSinkCreate(uint8_t *pParam);
static void APP_SendDataSinkRelease(uint8_t *pParam);
#if gKBD_KeysCount_c > 1
static void APP_SendLedRgbOn(uint8_t *pParam);
static void APP_SendLedRgbOff(uint8_t *pParam);
static void APP_SendLedFlash(uint8_t *pParam);
static void APP_SendLedColorWheel(uint8_t *pParam);
#endif
static void APP_LocalDataSinkRelease(uint8_t *pParam);
static void APP_ProcessLedCmd(uint8_t *pCommand, uint8_t dataLen);
static void APP_CoapGenericCallback(coapSessionStatus_t sessionStatus, uint8_t *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapLedCb(coapSessionStatus_t sessionStatus, uint8_t *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapTempCb(coapSessionStatus_t sessionStatus, uint8_t *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapSinkCb(coapSessionStatus_t sessionStatus, uint8_t *pData, coapSession_t *pSession, uint32_t dataLen);
static void App_RestoreLeaderLed(uint8_t *param);

// LAB
static void APP_CoapResource1Cb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapResource2Cb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapTeam9Cb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_CoapAccelCb(coapSessionStatus_t sessionStatus, void *pData, coapSession_t *pSession, uint32_t dataLen);

#if LARGE_NETWORK
static void APP_CoapResetToFactoryDefaultsCb(coapSessionStatus_t sessionStatus, uint8_t *pData, coapSession_t *pSession, uint32_t dataLen);
static void APP_SendResetToFactoryCommand(uint8_t *param);
#endif
#if APP_AUTOSTART
static void APP_AutoStart(void *param);
static void APP_AutoStartCb(void *param);
#endif


// Practica - parte 2
void BOARD_I2C_ReleaseBus(void);

/*==================================================================================================
Public global variables declarations
==================================================================================================*/
const coapUriPath_t gAPP_LED_URI_PATH  = {SizeOfString(APP_LED_URI_PATH), (uint8_t *)APP_LED_URI_PATH};
const coapUriPath_t gAPP_TEMP_URI_PATH = {SizeOfString(APP_TEMP_URI_PATH), (uint8_t *)APP_TEMP_URI_PATH};
const coapUriPath_t gAPP_SINK_URI_PATH = {SizeOfString(APP_SINK_URI_PATH), (uint8_t *)APP_SINK_URI_PATH};

//LAB
const coapUriPath_t gAPP_RESOURCE1_URI_PATH = {SizeOfString(APP_RESOURCE1_URI_PATH), APP_RESOURCE1_URI_PATH};
const coapUriPath_t gAPP_RESOURCE2_URI_PATH = {SizeOfString(APP_RESOURCE2_URI_PATH), APP_RESOURCE2_URI_PATH};
//practica
const coapUriPath_t gAPP_TEAM9_URI_PATH		= {SizeOfString(APP_TEAM9_URI_PATH), APP_TEAM9_URI_PATH};
const coapUriPath_t gAPP_ACCEL_URI_PATH		= {SizeOfString(APP_ACCEL_URI_PATH), APP_ACCEL_URI_PATH};

#if LARGE_NETWORK
const coapUriPath_t gAPP_RESET_URI_PATH = {SizeOfString(APP_RESET_TO_FACTORY_URI_PATH), (uint8_t *)APP_RESET_TO_FACTORY_URI_PATH};
#endif

/* Application state/mode */
appDeviceState_t gAppDeviceState[THR_MAX_INSTANCES];
appDeviceMode_t gAppDeviceMode[THR_MAX_INSTANCES];

/* Flag used to stop the attaching retries */
bool_t gbRetryInterrupt = TRUE;

bool_t gbCreateNetwork = FALSE;

/* CoAP instance */
uint8_t mAppCoapInstId = THR_ALL_FFs8;
/* Destination address for CoAP commands */
ipAddr_t gCoapDestAddress;

/* Application timer Id */
tmrTimerID_t mAppTimerId = gTmrInvalidTimerID_c;

#if APP_AUTOSTART
tmrTimerID_t tmrStartApp = gTmrInvalidTimerID_c;
#endif

uint32_t leaderLedTimestamp = 0;

/* Pointer application task message queue */
taskMsgQueue_t *mpAppThreadMsgQueue = NULL;

extern bool_t gEnable802154TxLed;

// Practica - Parte 2
i2c_master_handle_t g_MasterHandle;
/* FXOS device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

fxos_handle_t fxosHandle;
fxos_data_t sensorData;
i2c_master_config_t i2cConfig;
uint8_t sensorRange = 0;
uint8_t dataScale = 0;
uint32_t i2cSourceClock;
int16_t xData, yData, zData;
int16_t xAngle, yAngle;
uint8_t i = 0;
uint8_t regResult = 0;
uint8_t array_addr_size = 0;
bool_t foundDevice = false;

/*==================================================================================================
Public functions
==================================================================================================*/

// Practica - Parte 2

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

/* Initialize timer module */
static void Timer_Init(void)
{
    /* convert to match type of data */
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL;
    tpmParam[0].level = kTPM_LowTrue;
    tpmParam[0].dutyCyclePercent = 0U;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL;
    tpmParam[1].level = kTPM_LowTrue;
    tpmParam[1].dutyCyclePercent = 0U;

    /* Initialize TPM module */
    TPM_GetDefaultConfig(&tpmInfo);
    TPM_Init(BOARD_TIMER_BASEADDR, &tpmInfo);

    CLOCK_SetTpmClock(TIMER_CLOCK_MODE);

    TPM_SetupPwm(BOARD_TIMER_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 24000U, BOARD_TIMER_SOURCE_CLOCK);
    TPM_StartTimer(BOARD_TIMER_BASEADDR, kTPM_SystemClock);
}

/* Update the duty cycle of an active pwm signal */
static void Board_UpdatePwm(uint16_t x, uint16_t y)
{
    /* Updated duty cycle */
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, x);
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, y);
}

/*!*************************************************************************************************
\fn     void APP_Init(void)
\brief  This function is used to initialize application.
 ***************************************************************************************************/
void APP_Init
(
		void
)
{
	/* Initialize pointer to application task message queue */
	mpAppThreadMsgQueue = &appThreadMsgQueue;

	/* Initialize main thread message queue */
	ListInit(&appThreadMsgQueue.msgQueue,APP_MSG_QUEUE_SIZE);

	/* Set default device mode/state */
	APP_SetState(gThrDefaultInstanceId_c, gDeviceState_FactoryDefault_c);
	APP_SetMode(gThrDefaultInstanceId_c, gDeviceMode_Configuration_c);

	/* Initialize keyboard handler */
	pfAppKeyboardHandler = App_HandleKeyboard;

	/* Use one instance ID for application */
	mThrInstanceId = gThrDefaultInstanceId_c;

	// Practica
	timerTask_Init();

	// Practica - Parte 2
    BOARD_InitPins();
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();

    i2cSourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);
    fxosHandle.base = BOARD_ACCEL_I2C_BASEADDR;
    fxosHandle.i2cHandle = &g_MasterHandle;

    I2C_MasterGetDefaultConfig(&i2cConfig);
    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &i2cConfig, i2cSourceClock);
    I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_MasterHandle, NULL, NULL);

    /* Find sensor devices */
    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
    for (i = 0; i < array_addr_size; i++)
    {
        fxosHandle.xfer.slaveAddress = g_accel_address[i];
        if (FXOS_ReadReg(&fxosHandle, WHO_AM_I_REG, &regResult, 1) == kStatus_Success)
        {
            foundDevice = true;
            break;
        }
        if ((i == (array_addr_size - 1)) && (!foundDevice))
        {
            PRINTF("\r\nDo not found sensor device\r\n");
            while (1)
            {
            };
        }
    }

    /* Init accelerometer sensor */
    if (FXOS_Init(&fxosHandle) != kStatus_Success)
    {
        return -1;
    }

    /* Get sensor range */
    if (FXOS_ReadReg(&fxosHandle, XYZ_DATA_CFG_REG, &sensorRange, 1) != kStatus_Success)
    {
        return -1;
    }
    if (sensorRange == 0x00)
    {
        dataScale = 2U;
    }
    else if (sensorRange == 0x01)
    {
        dataScale = 4U;
    }
    else if (sensorRange == 0x10)
    {
        dataScale = 8U;
    }
    else
    {
    }
    /* Init timer */
    Timer_Init();

#if THR_ENABLE_EVENT_MONITORING
	/* Initialize event monitoring */
	APP_InitEventMonitor(mThrInstanceId);
#endif

	if(gThrStatus_Success_c == THR_StartInstance(mThrInstanceId, pStackCfg[0]))
	{
		/* Initialize CoAP demo */
		APP_InitCoapDemo();

#if USE_TEMPERATURE_SENSOR
		/* Initialize Temperature sensor/ADC module*/
		APP_InitADC(ADC_0);
#endif

#if THREAD_USE_THCI && THR_ENABLE_MGMT_DIAGNOSTICS
		(void)MgmtDiagnostic_RegisterAppCb(THCI_MgmtDiagnosticAppCb);
#endif

#if THREAD_USE_SHELL && SOCK_DEMO
		/* Initialize use sockets - used from shell */
		APP_InitUserSockets(mpAppThreadMsgQueue);
#endif

#if APP_AUTOSTART
		tmrStartApp = TMR_AllocateTimer();

		if(tmrStartApp != gTmrInvalidTimerID_c)
		{
			uint32_t jitterTime = NWKU_GetRandomNoFromInterval(gAppFactoryResetTimeoutMin_c,
					gAppFactoryResetTimeoutMax_c);
			TMR_StartSingleShotTimer(tmrStartApp, jitterTime, APP_AutoStartCb, NULL);
		}
#endif
	}
}

/*!*************************************************************************************************
\fn     void App_Handler(void)
\brief  Application Handler. In this configuration is called on the task with the lowest priority
 ***************************************************************************************************/
void APP_Handler
(
		void
)
{
	bool_t handleMsg = TRUE;

	while(handleMsg == TRUE)
	{
		handleMsg = NWKU_MsgHandler(&appThreadMsgQueue);
		/* For BareMetal break the while(1) after 1 run */
		if(!gUseRtos_c && MSG_Pending(&appThreadMsgQueue.msgQueue))
		{
			(void)OSA_EventSet(appThreadMsgQueue.taskEventId, NWKU_GENERIC_MSG_EVENT);
			break;
		}
	}
}

/*!*************************************************************************************************
\fn     void APP_NwkScanHandler(void *param)
\brief  This function is used to handle network scan results in asynchronous mode.

\param  [in]    param    Pointer to stack event
 ***************************************************************************************************/
void APP_NwkScanHandler
(
		void *param
)
{
	thrEvmParams_t *pEventParams = (thrEvmParams_t *)param;
	thrNwkScanResults_t *pScanResults = &pEventParams->pEventData->nwkScanCnf;

	/* Handle the network scan result here */
	if(pScanResults)
	{
#if THREAD_USE_SHELL
		SHELL_NwkScanPrint(pScanResults);
#endif
		MEM_BufferFree(pScanResults);
	}
	/* Free Event Buffer */
	MEM_BufferFree(pEventParams);
}

/*!*************************************************************************************************
\fn     void Stack_to_APP_Handler(void *param)
\brief  This function is used to handle stack events in asynchronous mode.

\param  [in]    param    Pointer to stack event
 ***************************************************************************************************/
void Stack_to_APP_Handler
(
		void *param
)
{
	thrEvmParams_t *pEventParams = (thrEvmParams_t *)param;

	/* Uncomment for multicast address */
	//ipAddr_t mCastGroup = MCAST_3EAD_ADDRESS;

	switch(pEventParams->code)
	{
	case gThrEv_GeneralInd_ResetToFactoryDefault_c:
		App_UpdateStateLeds(gDeviceState_FactoryDefault_c);
		break;

	case gThrEv_GeneralInd_InstanceRestoreStarted_c:
	case gThrEv_GeneralInd_ConnectingStarted_c:
		APP_SetMode(mThrInstanceId, gDeviceMode_Configuration_c);
		App_UpdateStateLeds(gDeviceState_JoiningOrAttaching_c);
		gEnable802154TxLed = FALSE;
		break;

	case gThrEv_NwkJoinCnf_Success_c:
	case gThrEv_NwkJoinCnf_Failed_c:
		APP_JoinEventsHandler(pEventParams->code);
		break;

	case gThrEv_GeneralInd_Connected_c:
		App_UpdateStateLeds(gDeviceState_NwkConnected_c);
		/* Set application CoAP destination to all nodes on connected network */
		THR_GetIP6Addr(mThrInstanceId, gAllThreadNodes_c, &gCoapDestAddress, NULL);
		APP_SetMode(mThrInstanceId, gDeviceMode_Application_c);
		mFirstPushButtonPressed  = FALSE;
		/* Synchronize server data */
		THR_BrPrefixAttrSync(mThrInstanceId);
		/* Enable LED for 80215.4 tx activity */
		gEnable802154TxLed = TRUE;
		/* Uncomment to register multicast address */
		//IP_IF_AddMulticastGroup6(gIpIfSlp0_c, &mCastGroup);

		// Practica
		timer1s_Start();
		break;

	case gThrEv_GeneralInd_RequestRouterId_c:
		gEnable802154TxLed = FALSE;
		break;

	case gThrEv_GeneralInd_RouterIdAssigned_c:
		break;

	case gThrEv_GeneralInd_ConnectingDeffered_c:
		APP_SetMode(mThrInstanceId, gDeviceMode_Configuration_c);
		gEnable802154TxLed = FALSE;
		App_UpdateStateLeds(gDeviceState_NwkOperationPending_c);
		break;

	case gThrEv_GeneralInd_ConnectingFailed_c:
	case gThrEv_GeneralInd_Disconnected_c:
		APP_SetMode(mThrInstanceId, gDeviceMode_Configuration_c);
		App_UpdateStateLeds(gDeviceState_NwkFailure_c);
		break;

	case gThrEv_GeneralInd_DeviceIsLeader_c:
		App_UpdateStateLeds(gDeviceState_Leader_c);
		gEnable802154TxLed = TRUE;
#if !LARGE_NETWORK
		/* Auto start commissioner for the partition for demo purposes */
		MESHCOP_StartCommissioner(pEventParams->thrInstId);
#endif
		break;

	case gThrEv_GeneralInd_DeviceIsRouter_c:
		App_UpdateStateLeds(gDeviceState_ActiveRouter_c);
		gEnable802154TxLed = TRUE;

#if UDP_ECHO_PROTOCOL
		ECHO_ProtocolInit(mpAppThreadMsgQueue);
#endif
		break;

	case gThrEv_GeneralInd_DevIsREED_c:
		App_UpdateStateLeds(gDeviceState_NwkConnected_c);
		gEnable802154TxLed = TRUE;
		break;

#if gLpmIncluded_d
	case gThrEv_GeneralInd_AllowDeviceToSleep_c:
		PWR_AllowDeviceToSleep();
		break;

	case gThrEv_GeneralInd_DisallowDeviceToSleep_c:
		PWR_DisallowDeviceToSleep();
		break;
#endif
	default:
		break;
	}

	/* Free event buffer */
	MEM_BufferFree(pEventParams->pEventData);
	MEM_BufferFree(pEventParams);
}

/*!*************************************************************************************************
\fn     void APP_Commissioning_Handler(void *param)
\brief  This function is used to handle Commissioning events in synchronous mode.

\param  [in]    param    Pointer to Commissioning event
 ***************************************************************************************************/
void APP_Commissioning_Handler
(
		void *param
)
{
	thrEvmParams_t *pEventParams = (thrEvmParams_t *)param;

	switch(pEventParams->code)
	{
	/* Joiner Events */
	case gThrEv_MeshCop_JoinerDiscoveryStarted_c:
		break;
	case gThrEv_MeshCop_JoinerDiscoveryFailed_c:
		break;
	case gThrEv_MeshCop_JoinerDiscoveryFailedFiltered_c:
		break;
	case gThrEv_MeshCop_JoinerDiscoverySuccess_c:
		break;
	case gThrEv_MeshCop_JoinerDtlsSessionStarted_c:
		App_UpdateStateLeds(gDeviceState_JoiningOrAttaching_c);
		break;
	case gThrEv_MeshCop_JoinerDtlsError_c:
	case gThrEv_MeshCop_JoinerError_c:
		App_UpdateStateLeds(gDeviceState_FactoryDefault_c);
		break;
	case gThrEv_MeshCop_JoinerAccepted_c:
		break;

		/* Commissioner Events(event set applies for all Commissioners: on-mesh, external, native) */
	case gThrEv_MeshCop_CommissionerPetitionStarted_c:
		break;
	case gThrEv_MeshCop_CommissionerPetitionAccepted_c:
	{
		uint8_t aDefaultEui[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
		thrOctet32_t defaultPskD = THR_PSK_D;

		MESHCOP_AddExpectedJoiner(mThrInstanceId, aDefaultEui, defaultPskD.aStr, defaultPskD.length, TRUE);
		MESHCOP_SyncSteeringData(mThrInstanceId, gMeshcopEuiMaskAllFFs_c);
		break;
	}
	case gThrEv_MeshCop_CommissionerPetitionRejected_c:
		break;
	case gThrEv_MeshCop_CommissionerPetitionError_c:
		break;
	case gThrEv_MeshCop_CommissionerKeepAliveSent_c:
		break;
	case gThrEv_MeshCop_CommissionerError_c:
		break;
	case gThrEv_MeshCop_CommissionerJoinerDtlsSessionStarted_c:
		break;
	case gThrEv_MeshCop_CommissionerJoinerDtlsError_c:
		break;
	case gThrEv_MeshCop_CommissionerJoinerAccepted_c:
		break;
	case gThrEv_MeshCop_CommissionerNwkDataSynced_c:
		break;
	}

	/* Free event buffer */
	MEM_BufferFree(pEventParams);
}

/*!*************************************************************************************************
\fn     void App_RestoreLeaderLedCb(void *param)
\brief  Called in Application state to restore leader LED.

\param  [in]    param    Not used
 ***************************************************************************************************/
void App_RestoreLeaderLedCb
(
		void *param
)
{
	(void)NWKU_SendMsg(App_RestoreLeaderLed, NULL, mpAppThreadMsgQueue);
}

/*==================================================================================================
Private functions
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     static void APP_InitCoapDemo(void)
\brief  Initialize CoAP demo.
 ***************************************************************************************************/
static void APP_InitCoapDemo
(
		void
)
{
	coapRegCbParams_t cbParams[] =  {{APP_CoapLedCb,  (coapUriPath_t *)&gAPP_LED_URI_PATH},
			{APP_CoapTempCb, (coapUriPath_t *)&gAPP_TEMP_URI_PATH},

			//{APP_CoapResource1Cb, (coapUriPath_t*)&gAPP_TEAM9_URI_PATH},  // LAB
			{APP_CoapResource2Cb, (coapUriPath_t*)&gAPP_RESOURCE2_URI_PATH},  // LAB
			{APP_CoapTeam9Cb, (coapUriPath_t*)&gAPP_TEAM9_URI_PATH},//PRACTICA
			{APP_CoapAccelCb, (coapUriPath_t*)&gAPP_ACCEL_URI_PATH},//PRACTICA


#if LARGE_NETWORK
			{APP_CoapResetToFactoryDefaultsCb, (coapUriPath_t *)&gAPP_RESET_URI_PATH},
#endif
			{APP_CoapSinkCb, (coapUriPath_t *)&gAPP_SINK_URI_PATH}};
	/* Register Services in COAP */
	sockaddrStorage_t coapParams = {0};

	NWKU_SetSockAddrInfo(&coapParams, NULL, AF_INET6, COAP_DEFAULT_PORT, 0, gIpIfSlp0_c);
	mAppCoapInstId = COAP_CreateInstance(NULL, &coapParams, (coapRegCbParams_t *)cbParams,
			NumberOfElements(cbParams));
}

/*!*************************************************************************************************
\private
\fn     static void APP_ThrNwkJoin(uint8_t *param)
\brief  Start the joining procedure.

\param  [in]    param    Not used
 ***************************************************************************************************/
static void APP_ThrNwkJoin
(
		uint8_t *param
)
{
	if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
	{
		/* User can treat join failure according to their application */
	}
}

/*!*************************************************************************************************
\private
\fn     static void App_JoinTimerCallback(void *param)
\brief  Join timer callback.

\param  [in]    param    Not used
 ***************************************************************************************************/
static void App_JoinTimerCallback
(
		void *param
)
{
	if(mFirstPushButtonPressed)
	{
		mJoiningIsAppInitiated = TRUE;
		TMR_FreeTimer(mAppTimerId);
		mAppTimerId = gTmrInvalidTimerID_c;
		(void)NWKU_SendMsg(APP_ThrNwkJoin, NULL, mpAppThreadMsgQueue);
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_ConfigModeSwShortPressHandler(uint32_t keyEvent)
\brief  This is a handler for  KBD module - short press events. Device is in configuration mode.

\param  [in]    keyEvent    The keyboard module event
 ***************************************************************************************************/
static void APP_ConfigModeSwShortPressHandler
(
		uint32_t keyEvent
)
{
	(void)keyEvent;

	if((APP_GetState(mThrInstanceId) == gDeviceState_FactoryDefault_c) ||
			(APP_GetState(mThrInstanceId) == gDeviceState_NwkFailure_c) ||
			(APP_GetState(mThrInstanceId) == gDeviceState_NwkOperationPending_c))
	{
		App_UpdateStateLeds(gDeviceState_JoiningOrAttaching_c);
		mFirstPushButtonPressed = TRUE;

		if(mAppTimerId == gTmrInvalidTimerID_c)
		{
			mAppTimerId = TMR_AllocateTimer();
		}

		/* Validate application timer Id */
		if(mAppTimerId != gTmrInvalidTimerID_c)
		{
			/* Start the application timer. Wait gAppJoinTimeout_c to start the joining procedure */
			TMR_StartSingleShotTimer(mAppTimerId, gAppJoinTimeout_c, App_JoinTimerCallback, NULL);
		}
		else
		{
			mJoiningIsAppInitiated = TRUE;
			/* No timer available - try to join the network */
			if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
			{
				/* User can treat join failure according to their application */
			}
		}
	}
	/* Double press */
	else if(mFirstPushButtonPressed)
	{
		/* Reset */
		mFirstPushButtonPressed = FALSE;

		if((mJoiningIsAppInitiated == FALSE) &&
				(!THR_GetAttr_IsDevConnected(mThrInstanceId)))
		{
			if(mAppTimerId != gTmrInvalidTimerID_c)
			{
				TMR_FreeTimer(mAppTimerId);
				mAppTimerId = gTmrInvalidTimerID_c;
			}

			App_UpdateStateLeds(gDeviceState_Leader_c);

			/* Create the network */
			(void)THR_NwkCreate(mThrInstanceId);
		}
		else
		{
			/* Create network */
			gbCreateNetwork = TRUE;
			/* Device will create the network after receiving the next gThrEv_NwkJoinCnf_Failed_c event */
		}
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_ConfigModeHandleKeyboard(uint32_t keyEvent)
\brief  This is a handler for KBD module events. Device is in configuration mode.

\param  [in]    keyEvent   The keyboard module event
 ***************************************************************************************************/
static void APP_ConfigModeHandleKeyboard
(
		uint32_t keyEvent
)
{
	switch(keyEvent)
	{
	case gKBD_EventPB1_c:
#if gKBD_KeysCount_c > 1
	case gKBD_EventPB2_c:
	case gKBD_EventPB3_c:
	case gKBD_EventPB4_c:
#endif
		APP_ConfigModeSwShortPressHandler(keyEvent);
		break;
	case gKBD_EventLongPB1_c:
#if gKBD_KeysCount_c > 1
	case gKBD_EventLongPB2_c:
	case gKBD_EventLongPB3_c:
	case gKBD_EventLongPB4_c:
#endif
		break;
		/* Factory reset */
	case gKBD_EventVeryLongPB1_c:
#if gKBD_KeysCount_c > 1
	case gKBD_EventVeryLongPB2_c:
	case gKBD_EventVeryLongPB3_c:
	case gKBD_EventVeryLongPB4_c:
#endif
		THR_FactoryReset();
		break;
	default:
		break;
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_AppModeHandleKeyboard(uint32_t keyEvent)
\brief  This is a handler for KBD module events. Device is in application mode.

\param  [in]    keyEvent    The keyboard module event
 ***************************************************************************************************/
static void APP_AppModeHandleKeyboard
(
		uint32_t keyEvent
)
{
	switch(keyEvent)
	{
	case gKBD_EventPB1_c:
		/* Data sink create */
		(void)NWKU_SendMsg(APP_SendDataSinkCreate, NULL, mpAppThreadMsgQueue);
		break;
#if gKBD_KeysCount_c > 1
	case gKBD_EventPB2_c:
		/* Report temperature */
		(void)NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);
		break;
	case gKBD_EventPB3_c:
		/* Remote led RGB - on */
		(void)NWKU_SendMsg(APP_SendLedRgbOn, NULL, mpAppThreadMsgQueue);
		break;
	case gKBD_EventPB4_c:
		/* Remote led RGB - off */
		(void)NWKU_SendMsg(APP_SendLedRgbOff, NULL, mpAppThreadMsgQueue);
		break;
#endif
	case gKBD_EventLongPB1_c:
		/* Remote data sink release */
		(void)NWKU_SendMsg(APP_SendDataSinkRelease, NULL, mpAppThreadMsgQueue);
		break;
#if gKBD_KeysCount_c > 1
	case gKBD_EventLongPB2_c:
		/* Local data sink release */
		(void)NWKU_SendMsg(APP_LocalDataSinkRelease, NULL, mpAppThreadMsgQueue);
		break;
	case gKBD_EventLongPB3_c:
		/* Remote led flash */
		(void)NWKU_SendMsg(APP_SendLedFlash, NULL, mpAppThreadMsgQueue);
		break;
	case gKBD_EventLongPB4_c:
		/* Remote led - color wheel */
		(void)NWKU_SendMsg(APP_SendLedColorWheel, NULL, mpAppThreadMsgQueue);
		break;
#endif
	case gKBD_EventVeryLongPB1_c:
#if gKBD_KeysCount_c > 1
	case gKBD_EventVeryLongPB4_c:
#if LARGE_NETWORK
		/* OTA factory reset */
		(void)NWKU_SendMsg(APP_SendResetToFactoryCommand, NULL, mpAppThreadMsgQueue);
		break;
#endif
	case gKBD_EventVeryLongPB3_c:
	case gKBD_EventVeryLongPB2_c:
#endif
		/* Factory reset */
		THR_FactoryReset();
		break;
	default:
		break;
	}
}

/*!*************************************************************************************************
\private
\fn     static void App_HandleKeyboard(uint8_t *param)
\brief  This is a handler for KBD module events.

\param  [in]    param    The keyboard module event
 ***************************************************************************************************/
static void App_HandleKeyboard
(
		uint8_t *param
)
{
	uint32_t events = (uint32_t)(param);

	if(APP_GetMode(mThrInstanceId) == gDeviceMode_Configuration_c)
	{
		/* Device is in configuration mode */
		APP_ConfigModeHandleKeyboard(events);
	}
	else
	{
		/* Device is in application mode */
		APP_AppModeHandleKeyboard(events);
	}
}

/*!*************************************************************************************************
\private
\fn     static void App_UpdateLedState(appDeviceState_t deviceState)
\brief  Called when Application state and LEDs must be updated.

\param  [in]    deviceState    The current device state
 ***************************************************************************************************/
static void App_UpdateStateLeds
(
		appDeviceState_t deviceState
)
{
	/* If the user presses a button different than the LED off button, reset timestamp */
	if((THR_GetAttr_DeviceRole(mThrInstanceId) == gThrDevRole_Leader_c) &&
			(APP_GetState(mThrInstanceId) != gDeviceState_AppLedOff_c) &&
			(leaderLedTimestamp != 0))
	{
		leaderLedTimestamp = 0;
	}

	APP_SetState(mThrInstanceId, deviceState);
	Led_SetState(APP_GetMode(mThrInstanceId), APP_GetState(mThrInstanceId));
}

/*!*************************************************************************************************
\private
\fn     static void APP_JoinEventsHandler(thrEvCode_t evCode)
\brief  This function is used to the handle join failed event.

\param  [in]    evCode    Event code
 ***************************************************************************************************/
static void APP_JoinEventsHandler
(
		thrEvCode_t evCode
)
{
	if(mJoiningIsAppInitiated)
	{
		if(evCode == gThrEv_NwkJoinCnf_Failed_c)
		{
			if(gbRetryInterrupt && !gbCreateNetwork)
			{
				mJoiningIsAppInitiated = TRUE;

				/* Retry to join the network */
				if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
				{
					/* User can treat join failure according to their application */
				}
				return;
			}
			else if(gbCreateNetwork)
			{
				/* Create the network */
				(void)THR_NwkCreate(mThrInstanceId);
			}
			mJoiningIsAppInitiated = FALSE;
		}
		else if(evCode == gThrEv_NwkJoinCnf_Success_c)
		{
			mJoiningIsAppInitiated = FALSE;
		}
	}
}

/*==================================================================================================
  Coap Demo functions:
==================================================================================================*/
/*!*************************************************************************************************
\private
\fn     static void APP_CoapGenericCallback(sessionStatus sessionStatus, uint8_t *pData,
                                            coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the generic callback function for CoAP message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
 ***************************************************************************************************/
static void APP_CoapGenericCallback
(
		coapSessionStatus_t sessionStatus,
		uint8_t *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	/* If no ACK was received, try again */
	if(sessionStatus == gCoapFailure_c)
	{
		if(FLib_MemCmp(pSession->pUriPath->pUriPath, (coapUriPath_t *)&gAPP_TEMP_URI_PATH.pUriPath,
				pSession->pUriPath->length))
		{
			(void)NWKU_SendMsg(APP_ReportTemp, NULL, mpAppThreadMsgQueue);
		}
	}
	/* Process data, if any, for sessionStatus != gCoapFailure_c */
	else
	{
		if (pSession->msgType == gCoapNonConfirmable_c)
		{
			//Process data
		}
		else if ((pSession->msgType == gCoapConfirmable_c) && (sessionStatus == gCoapSuccess_c))
		{
			//Process data
		}
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_ReportTemp(uint8_t *pParam)
\brief  This open a socket and report the temperature to gCoapDestAddress.

\param  [in]    pParam    Not used
 ***************************************************************************************************/
static void APP_ReportTemp
(
		uint8_t *pParam
)
{
	coapSession_t *pSession = NULL;
	/* Get Temperature */
	uint8_t *pTempString = App_GetTempDataString();
	uint32_t ackPloadSize;

	if(!IP_IF_IsMyAddr(gIpIfSlp0_c, &gCoapDestAddress))
	{
		pSession = COAP_OpenSession(mAppCoapInstId);

		if(NULL != pSession)
		{
			coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;

			pSession->pCallback = NULL;
			FLib_MemCpy(&pSession->remoteAddrStorage.ss_addr, &gCoapDestAddress, sizeof(ipAddr_t));
			ackPloadSize = strlen((char *)pTempString);
			pSession->pUriPath = (coapUriPath_t *)&gAPP_TEMP_URI_PATH;

			if(!IP6_IsMulticastAddr(&gCoapDestAddress))
			{
				coapMessageType = gCoapMsgTypeConPost_c;
				COAP_SetCallback(pSession, APP_CoapGenericCallback);
			}

			COAP_Send(pSession, coapMessageType, pTempString, ackPloadSize);
		}
	}
	/* Print temperature in shell */
	shell_write("\r");
	shell_write((char *)pTempString);
	shell_refresh();
	MEM_BufferFree(pTempString);
}

/*!*************************************************************************************************
\private
\fn     static nwkStatus_t APP_SendDataSinkCommand(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to send a Data Sink command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pCommand       Pointer to command data
\param  [in]    dataLen        Data length

\return         nwkStatus_t    Status of the command
 ***************************************************************************************************/
static nwkStatus_t APP_SendDataSinkCommand
(
		uint8_t *pCommand,
		uint8_t dataLen
)
{
	nwkStatus_t status = gNwkStatusFail_c;
	coapSession_t *pSession = COAP_OpenSession(mAppCoapInstId);

	if(pSession)
	{
		ipAddr_t coapDestAddress = APP_DEFAULT_DEST_ADDR;

		pSession->pCallback = NULL;
		FLib_MemCpy(&pSession->remoteAddrStorage.ss_addr, &coapDestAddress, sizeof(ipAddr_t));
		pSession->pUriPath = (coapUriPath_t*)&gAPP_SINK_URI_PATH;
		status = COAP_Send(pSession, gCoapMsgTypeNonPost_c, pCommand, dataLen);
	}

	return status;
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendDataSinkCreate(uint8_t *pParam)
\brief  This function is used to send a Data Sink Create command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pParam    Not used
 ***************************************************************************************************/
static void APP_SendDataSinkCreate
(
		uint8_t *pParam
)
{
	uint8_t aCommand[] = {"create"};

	/* Send command over the air */
	if(APP_SendDataSinkCommand(aCommand, sizeof(aCommand)) == gNwkStatusSuccess_c)
	{
		/* Local data sink create */
		(void)THR_GetIP6Addr(mThrInstanceId, gMLEIDAddr_c, &gCoapDestAddress, NULL);
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendDataSinkRelease(uint8_t *pParam)
\brief  This function is used to send a Data Sink Release command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pParam    Pointer to stack event
 ***************************************************************************************************/
static void APP_SendDataSinkRelease
(
		uint8_t *pParam
)
{
	uint8_t aCommand[] = {"release"};

	/* Send command over the air */
	if(APP_SendDataSinkCommand(aCommand, sizeof(aCommand)) == gNwkStatusSuccess_c)
	{
		/* Local data sink release */
		APP_LocalDataSinkRelease(pParam);
	}
}

#if gKBD_KeysCount_c > 1
/*!*************************************************************************************************
\private
\fn     static void APP_SendLedCommand(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to send a Led command to gCoapDestAddress.

\param  [in]    pCommand    Pointer to command data
\param  [in]    dataLen     Data length
 ***************************************************************************************************/
static void APP_SendLedCommand
(
		uint8_t *pCommand,
		uint8_t dataLen
)
{
	if(!IP_IF_IsMyAddr(gIpIfSlp0_c, &gCoapDestAddress))
	{
		coapSession_t *pSession = COAP_OpenSession(mAppCoapInstId);

		if(pSession)
		{
			coapMsgTypesAndCodes_t coapMessageType = gCoapMsgTypeNonPost_c;

			pSession->pCallback = NULL;
			FLib_MemCpy(&pSession->remoteAddrStorage.ss_addr, &gCoapDestAddress, sizeof(ipAddr_t));
			pSession->pUriPath = (coapUriPath_t *)&gAPP_LED_URI_PATH;

			if(!IP6_IsMulticastAddr(&gCoapDestAddress))
			{
				coapMessageType = gCoapMsgTypeConPost_c;
				COAP_SetCallback(pSession, APP_CoapGenericCallback);
			}
			else
			{
				APP_ProcessLedCmd(pCommand, dataLen);
			}
			COAP_Send(pSession, coapMessageType, pCommand, dataLen);
		}
	}
	else
	{
		APP_ProcessLedCmd(pCommand, dataLen);
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedRgbOn(uint8_t *pParam)
\brief  This function is used to send a Led RGB On command over the air.

\param  [in]    pParam    Not used
 ***************************************************************************************************/
static void APP_SendLedRgbOn
(
		uint8_t *pParam
)
{
	uint8_t aCommand[] = {"rgb r000 g000 b000"};
	uint8_t redValue, greenValue, blueValue;

	/* Red value on: 0x01 - 0xFF */
	redValue = (uint8_t)NWKU_GetRandomNoFromInterval(0x01, THR_ALL_FFs8);

	/* Green value on: 0x01 - 0xFF */
	greenValue = (uint8_t)NWKU_GetRandomNoFromInterval(0x01, THR_ALL_FFs8);

	/* Blue value on: 0x01 - 0xFF */
	blueValue = (uint8_t)NWKU_GetRandomNoFromInterval(0x01, THR_ALL_FFs8);

	NWKU_PrintDec(redValue, aCommand + 5, 3, TRUE);     //aCommand + strlen("rgb r")
	NWKU_PrintDec(greenValue, aCommand + 10, 3, TRUE);  //aCommand + strlen("rgb r000 g")
	NWKU_PrintDec(blueValue, aCommand + 15, 3, TRUE);   //aCommand + strlen("rgb r000 g000 b")

	APP_SendLedCommand(aCommand, sizeof(aCommand));
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedRgbOff(uint8_t *pParam)
\brief  This function is used to send a Led RGB Off command over the air.

\param  [in]    pParam    Not used
 ***************************************************************************************************/
static void APP_SendLedRgbOff
(
		uint8_t *pParam
)
{
	uint8_t aCommand[] = {"rgb r000 g000 b000"};

	APP_SendLedCommand(aCommand, sizeof(aCommand));
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedFlash(uint8_t *pParam)
\brief  This function is used to send a Led flash command over the air.

\param  [in]    pParam    Not used
 ***************************************************************************************************/
static void APP_SendLedFlash
(
		uint8_t *pParam
)
{
	uint8_t aCommand[] = {"flash"};

	APP_SendLedCommand(aCommand, sizeof(aCommand));
}

/*!*************************************************************************************************
\private
\fn     static void APP_SendLedColorWheel(uint8_t *pParam)
\brief  This function is used to send a Led color wheel command over the air.

\param  [in]    pParam    Pointer to stack event
 ***************************************************************************************************/
static void APP_SendLedColorWheel
(
		uint8_t *pParam
)
{
	uint8_t aCommand[] = {"color wheel"};

	APP_SendLedCommand(aCommand, sizeof(aCommand));
}
#endif

/*!*************************************************************************************************
\private
\fn     static void APP_LocalDataSinkRelease(uint8_t *pParam)
\brief  This function is used to restore the default destination address for CoAP messages.

\param  [in]    pParam    Pointer to stack event
 ***************************************************************************************************/
static void APP_LocalDataSinkRelease
(
		uint8_t *pParam
)
{
	ipAddr_t defaultDestAddress = APP_DEFAULT_DEST_ADDR;

	FLib_MemCpy(&gCoapDestAddress, &defaultDestAddress, sizeof(ipAddr_t));
	(void)pParam;
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapLedCb(coapSessionStatus_t sessionStatus, uint8_t *pData,
                                  coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP LED message.
\brief  It performs the required operations and sends back a CoAP ACK message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
 ***************************************************************************************************/
static void APP_CoapLedCb
(
		coapSessionStatus_t sessionStatus,
		uint8_t *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	/* Process the command only if it is a POST method */
	if((pData) && (sessionStatus == gCoapSuccess_c) && (pSession->code == gCoapPOST_c))
	{
		APP_ProcessLedCmd(pData, dataLen);
	}

	/* Send the reply if the status is Success or Duplicate */
	if((gCoapFailure_c != sessionStatus) && (gCoapConfirmable_c == pSession->msgType))
	{
		/* Send CoAP ACK */
		COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_ProcessLedCmd(uint8_t *pCommand, uint8_t dataLen)
\brief  This function is used to process a LED command (on, off, flash, toggle, rgb, color wheel).

\param  [in]    pCommand    Pointer to command data
\param  [in]    dataLen     Data length
 ***************************************************************************************************/
static void APP_ProcessLedCmd
(
		uint8_t *pCommand,
		uint8_t dataLen
)
{
	/* Set mode state */
	APP_SetMode(mThrInstanceId, gDeviceMode_Application_c);
	mFirstPushButtonPressed  = FALSE;

	/* Process command */
	if(FLib_MemCmp(pCommand, "on", 2))
	{
		App_UpdateStateLeds(gDeviceState_AppLedOn_c);
	}
	else if(FLib_MemCmp(pCommand, "off", 3))
	{
		App_UpdateStateLeds(gDeviceState_AppLedOff_c);
	}
	else if(FLib_MemCmp(pCommand, "toggle", 6))
	{
		App_UpdateStateLeds(gDeviceState_AppLedToggle_c);
	}
	else if(FLib_MemCmp(pCommand, "flash", 5))
	{
		App_UpdateStateLeds(gDeviceState_AppLedFlash_c);
	}
	else if(FLib_MemCmp(pCommand, "rgb", 3))
	{
		char* p = (char *)pCommand + strlen("rgb");
		uint8_t redValue = 0, greenValue = 0, blueValue = 0;
		appDeviceState_t appState = gDeviceState_AppLedRgb_c;

		dataLen -= strlen("rgb");

		while(dataLen != 0)
		{
			if(*p == 'r')
			{
				p++;
				dataLen--;
				redValue = NWKU_atoi(p);
			}

			if(*p == 'g')
			{
				p++;
				dataLen--;
				greenValue = NWKU_atoi(p);
			}

			if(*p == 'b')
			{
				p++;
				dataLen--;
				blueValue = NWKU_atoi(p);
			}

			dataLen--;
			p++;
		}

		/* Update RGB values */
#if gLedRgbEnabled_d
		Led_UpdateRgbState(redValue, greenValue, blueValue);
#else
		appState = gDeviceState_AppLedOff_c;

		if(redValue || greenValue || blueValue)
		{
			appState = gDeviceState_AppLedOn_c;
		}
#endif
		App_UpdateStateLeds(appState);

		/* If device is leader and has received a RGB LED off command and there were no previous button presses */
		if((THR_GetAttr_DeviceRole(mThrInstanceId) == gThrDevRole_Leader_c) &&
				(!redValue && !greenValue && !blueValue) && (leaderLedTimestamp == 0))
		{
			leaderLedTimestamp = (TMR_GetTimestamp()/1000000) + gAppRestoreLeaderLedTimeout_c;
		}
	}
	else if(FLib_MemCmp(pCommand, "color wheel", 11))
	{
#if gLedRgbEnabled_d
		App_UpdateStateLeds(gDeviceState_AppLedColorWheel_c);
#else
		App_UpdateStateLeds(gDeviceState_AppLedFlash_c);
#endif
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapTempCb(coapSessionStatus_t sessionStatus, uint8_t *pData,
                                   coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP temperature message.
\brief  It sends the temperature value in a CoAP ACK message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
 ***************************************************************************************************/
static void APP_CoapTempCb
(
		coapSessionStatus_t sessionStatus,
		uint8_t *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	uint8_t *pTempString = NULL;
	uint32_t ackPloadSize = 0, maxDisplayedString = 10;

	/* Send CoAP ACK */
	if(gCoapGET_c == pSession->code)
	{
		/* Get Temperature */
		pTempString = App_GetTempDataString();
		ackPloadSize = strlen((char*)pTempString);
	}
	/* Do not parse the message if it is duplicated */
	else if((gCoapPOST_c == pSession->code) && (sessionStatus == gCoapSuccess_c))
	{
		if(NULL != pData)
		{
			char addrStr[INET6_ADDRSTRLEN];
			uint8_t temp[10];

			ntop(AF_INET6, (ipAddr_t*)&pSession->remoteAddrStorage.ss_addr, addrStr, INET6_ADDRSTRLEN);
			shell_write("\r");

			if(0 != dataLen)
			{
				/* Prevent from buffer overload */
				(dataLen >= maxDisplayedString) ? (dataLen = (maxDisplayedString - 1)) : (dataLen);
				temp[dataLen]='\0';
				FLib_MemCpy(temp,pData,dataLen);
				shell_printf((char*)temp);
			}
			shell_printf("\tFrom IPv6 Address: %s\n\r", addrStr);
			shell_refresh();
		}
	}

	if(gCoapConfirmable_c == pSession->msgType)
	{
		if(gCoapGET_c == pSession->code)
		{
			COAP_Send(pSession, gCoapMsgTypeAckSuccessContent_c, pTempString, ackPloadSize);
		}
		else
		{
			COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
		}
	}

	if(pTempString)
	{
		MEM_BufferFree(pTempString);
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapSinkCb(coapSessionStatus_t sessionStatus, uint8_t *pData,
                                   coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP sink message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
 ***************************************************************************************************/
static void APP_CoapSinkCb
(
		coapSessionStatus_t sessionStatus,
		uint8_t *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	/* Do not execute the command multiple times, if the received message is duplicated */
	if((pData) && (sessionStatus == gCoapSuccess_c))
	{
		/* Process command */
		if(FLib_MemCmp(pData, "create",6))
		{
			/* Data sink create */
			FLib_MemCpy(&gCoapDestAddress, &pSession->remoteAddrStorage.ss_addr, sizeof(ipAddr_t));
		}

		if(FLib_MemCmp(pData, "release",7))
		{
			/* Data sink release */
			APP_LocalDataSinkRelease(NULL);
		}
	}

	if(gCoapConfirmable_c == pSession->msgType)
	{
		/* Send CoAP ACK */
		COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, NULL, 0);
	}
}

/*!*************************************************************************************************
\private
\fn     static void App_RestoreLeaderLed(uint8_t *param)
\brief  Called in Application state to restore leader LED.

\param  [in]    param    Not used
 ***************************************************************************************************/
static void App_RestoreLeaderLed
(
		uint8_t *param
)
{
	App_UpdateStateLeds(gDeviceState_Leader_c);
}

/** LAB **/



static void APP_CoapResource1Cb
(
		coapSessionStatus_t sessionStatus,
		void *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	static uint8_t pMySessionPayload[3]={0x31,0x32,0x33};
	static uint32_t pMyPayloadSize=3;
	coapSession_t *pMySession = NULL;
	pMySession = COAP_OpenSession(mAppCoapInstId);
	COAP_AddOptionToList(pMySession, COAP_URI_PATH_OPTION, APP_RESOURCE1_URI_PATH,SizeOfString(APP_RESOURCE1_URI_PATH));

	shell_write("' team 9 debug ");

	if (gCoapConfirmable_c == pSession->msgType)
	{
		if (gCoapGET_c == pSession->code)
		{
			shell_write("'CON' packet received 'GET' with payload: ");
		}
		if (gCoapPOST_c == pSession->code)
		{
			shell_write("'CON' packet received 'POST' with payload: ");
		}
		if (gCoapPUT_c == pSession->code)
		{
			shell_write("'CON' packet received 'PUT' with payload: ");
		}
		if (gCoapFailure_c != sessionStatus)
		{
			COAP_Send(pSession, gCoapMsgTypeAckSuccessChanged_c, pMySessionPayload, pMyPayloadSize);
		}
	}

	else if(gCoapNonConfirmable_c == pSession->msgType)
	{
		if (gCoapGET_c == pSession->code)
		{
			shell_write("'NON' packet received 'GET' with payload: ");
		}
		if (gCoapPOST_c == pSession->code)
		{
			shell_write("'NON' packet received 'POST' with payload: ");
		}
		if (gCoapPUT_c == pSession->code)
		{
			shell_write("'NON' packet received 'PUT' with payload: ");
		}

	}
	shell_writeN(pData, dataLen);
	shell_write("\r\n");
	pMySession -> msgType=gCoapNonConfirmable_c;
	pMySession -> code= gCoapPOST_c;
	pMySession -> pCallback =NULL;
	FLib_MemCpy(&pMySession->remoteAddrStorage,&gCoapDestAddress,sizeof(ipAddr_t));
	COAP_Send(pMySession, gCoapMsgTypeNonPost_c, pMySessionPayload, pMyPayloadSize);
	shell_write("'NON' packet sent 'POST' with payload: ");
	shell_writeN((char*) pMySessionPayload, pMyPayloadSize);
	shell_write("\r\n");

}

static void APP_CoapResource2Cb
(
		coapSessionStatus_t sessionStatus,
		void *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	if (gCoapNonConfirmable_c == pSession->msgType)
	{
		shell_write("'NON' packet received 'POST' with payload: ");
		shell_writeN(pData, dataLen);
		shell_write("\r\n");
		//COAP_CloseSession(pSession);
	}
	if (gCoapConfirmable_c == pSession->msgType)
	{
		shell_write("'CON' packet received 'POST' with payload: ");
		shell_writeN(pData, dataLen);
		shell_write("\r\n");
		//COAP_CloseSession(pSession);
	}
}


static void APP_CoapAccelCb
(
		coapSessionStatus_t sessionStatus,
		void *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	static uint8_t pMySessionPayload[3]={0x31,0x32,0x33};
	static uint32_t pMyPayloadSize=3;
	coapSession_t *pMySession = NULL;
	pMySession = COAP_OpenSession(mAppCoapInstId);
	COAP_AddOptionToList(pMySession, COAP_URI_PATH_OPTION, APP_RESOURCE2_URI_PATH,SizeOfString(APP_RESOURCE2_URI_PATH));


	if (gCoapConfirmable_c == pSession->msgType)
	{
		shell_write("\r\n CON Requested URI at accel: ");

		if (gCoapGET_c == pSession->code)
		{
			shell_write("'CON' packet received 'GET' with payload: ");

			/// send reply message
			shell_write("\r\n");
			pMySession -> msgType=gCoapConfirmable_c;
			pMySession -> code= gCoapPOST_c;
			pMySession -> pCallback =NULL;
			FLib_MemCpy(&pMySession->remoteAddrStorage,&gCoapDestAddress,sizeof(ipAddr_t));

	        /* Get new accelerometer data. */
	        if (FXOS_ReadSensorData(&fxosHandle, &sensorData) != kStatus_Success)
	        {
	            return -1;
	        }

			uint8_t accel_reading[6];
			accel_reading[0] = sensorData.accelXMSB;
			accel_reading[1] = sensorData.accelXLSB;
			accel_reading[2] = sensorData.accelYMSB;
			accel_reading[3] = sensorData.accelYLSB;
			accel_reading[4] = sensorData.accelZMSB;
			accel_reading[5] = sensorData.accelZLSB;
			COAP_Send(pMySession, gCoapMsgTypeNonPost_c, accel_reading, 6);
	        xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
	        yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;
	        zData = (int16_t)((uint16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB) / 4U;
			shell_write("'CON' packet sent  with values X: ");
			shell_printf(" %i", xData);
			shell_printf(" Y: ");
			shell_printf("%i", yData);
			shell_printf(" Z: ");
			shell_printf("%i", zData);
			shell_write("\r\n");

		}
		if (gCoapPOST_c == pSession->code)
		{
			shell_write("'CON' packet received 'POST' with payload: ");
		}
		if (gCoapPUT_c == pSession->code)
		{
			shell_write("'CON' packet received 'PUT' with payload: ");
		}
		///** send ACK , as the message is CON */
		if (gCoapFailure_c != sessionStatus)
		{
			COAP_Send(pMySession, gCoapMsgTypeAckSuccessChanged_c, pMySessionPayload, pMyPayloadSize);

			//COAP_CloseSession(pSession);
		}
	}

	else if(gCoapNonConfirmable_c == pSession->msgType)
	{
		shell_write("\r\n NON Requested URI at accel: ");

		if (gCoapGET_c == pSession->code)
		{
			shell_write("'NON' packet received 'GET' with payload: ");


			/// send reply message
			shell_write("\r\n");
			pMySession -> msgType=gCoapNonConfirmable_c;
			pMySession -> code= gCoapPOST_c;
			pMySession -> pCallback =NULL;
			FLib_MemCpy(&pMySession->remoteAddrStorage,&gCoapDestAddress,sizeof(ipAddr_t));

	        /* Get new accelerometer data. */
	        if (FXOS_ReadSensorData(&fxosHandle, &sensorData) != kStatus_Success)
	        {
	            return -1;
	        }

			uint8_t accel_reading[6];
			accel_reading[0] = sensorData.accelXMSB;
			accel_reading[1] = sensorData.accelXLSB;
			accel_reading[2] = sensorData.accelYMSB;
			accel_reading[3] = sensorData.accelYLSB;
			accel_reading[4] = sensorData.accelZMSB;
			accel_reading[5] = sensorData.accelZLSB;
			COAP_Send(pMySession, gCoapMsgTypeNonPost_c, accel_reading, 6);
	        xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
	        yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;
	        zData = (int16_t)((uint16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB) / 4U;
			shell_write("'NON' packet sent  with values X: ");
			shell_printf(" %i", xData);
			shell_printf(" Y: ");
			shell_printf("%i", yData);
			shell_printf(" Z: ");
			shell_printf("%i", zData);
			shell_write("\r\n");
		}
		if (gCoapPOST_c == pSession->code)
		{
			shell_write("'NON' packet received 'POST' with payload: ");
		}
		if (gCoapPUT_c == pSession->code)
		{
			shell_write("'NON' packet received 'PUT' with payload: ");
		}

	}

	COAP_CloseSession(pSession);

}
static void APP_CoapTeam9Cb
(
		coapSessionStatus_t sessionStatus,
		void *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	static uint8_t pMySessionPayload[3]={0x31,0x32,0x33};
	static uint32_t pMyPayloadSize=3;
	coapSession_t *pMySession = NULL;
	pMySession = COAP_OpenSession(mAppCoapInstId);
	COAP_AddOptionToList(pMySession, COAP_URI_PATH_OPTION, APP_RESOURCE2_URI_PATH,SizeOfString(APP_RESOURCE2_URI_PATH));


	/** print on shell the shell address of the requester*/
	shell_printf("\r\n   Team 9  Received from %x%x::%x%x:%x%x::%x%x::%x%x", pSession->remoteAddrStorage.ss_addr[0],
			pSession->remoteAddrStorage.ss_addr[1],

			pSession->remoteAddrStorage.ss_addr[8],
			pSession->remoteAddrStorage.ss_addr[9],

			pSession->remoteAddrStorage.ss_addr[10],
			pSession->remoteAddrStorage.ss_addr[11],

			pSession->remoteAddrStorage.ss_addr[12],
			pSession->remoteAddrStorage.ss_addr[13],

			pSession->remoteAddrStorage.ss_addr[14]
	);

	if (gCoapConfirmable_c == pSession->msgType)
	{


		if (gCoapGET_c == pSession->code)
		{
			shell_write("'CON' packet received 'GET' with payload: ");
		}
		if (gCoapPOST_c == pSession->code)
		{
			shell_write("'CON' packet received 'POST' with payload: ");
		}
		if (gCoapPUT_c == pSession->code)
		{
			shell_write("'CON' packet received 'PUT' with payload: ");
		}
		///** send ACK , as the message is CON */
		if (gCoapFailure_c != sessionStatus)
		{
			COAP_Send(pMySession, gCoapMsgTypeAckSuccessChanged_c, pMySessionPayload, pMyPayloadSize);

			//COAP_CloseSession(pSession);
		}
	}

	else if(gCoapNonConfirmable_c == pSession->msgType)
	{
		shell_write("\r\n NON Requested URI at team 9: ");

		if (gCoapGET_c == pSession->code)
		{
			shell_write("'NON' packet received 'GET' with payload: ");


			/// send reply message
			shell_write("\r\n");
			pMySession -> msgType=gCoapNonConfirmable_c;
			pMySession -> code= gCoapPOST_c;
			pMySession -> pCallback =NULL;
			FLib_MemCpy(&pMySession->remoteAddrStorage,&gCoapDestAddress,sizeof(ipAddr_t));

			uint8_t counter  = getCounter();
			COAP_Send(pMySession, gCoapMsgTypeNonPost_c, &counter, 1);
			shell_write("'NON' packet sent  with counter value: ");
			shell_printf(" %i", counter);
			shell_write("\r\n");
		}
		if (gCoapPOST_c == pSession->code)
		{
			shell_write("'NON' packet received 'POST' with payload: ");
		}
		if (gCoapPUT_c == pSession->code)
		{
			shell_write("'NON' packet received 'PUT' with payload: ");
		}



	}

	COAP_CloseSession(pSession);

}


#if LARGE_NETWORK
/*!*************************************************************************************************
\private
\fn     static void APP_SendResetToFactoryCommand(uint8_t *pParam)
\brief  This function is used to send a Factory Reset command to APP_DEFAULT_DEST_ADDR.

\param  [in]    pParam    Pointer to stack event
 ***************************************************************************************************/
static void APP_SendResetToFactoryCommand
(
		uint8_t *param
)
{
	coapSession_t *pSession = COAP_OpenSession(mAppCoapInstId);

	if(pSession)
	{
		ipAddr_t coapDestAddress = APP_DEFAULT_DEST_ADDR;

		pSession->pCallback = NULL;
		FLib_MemCpy(&pSession->remoteAddrStorage.ss_addr, &coapDestAddress, sizeof(ipAddr_t));
		pSession->pUriPath = (coapUriPath_t *)&gAPP_RESET_URI_PATH;
		COAP_Send(pSession, gCoapMsgTypeNonPost_c, NULL, 0);
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_CoapResetToFactoryDefaultsCb(coapSessionStatus_t sessionStatus, uint8_t *pData,
                                                     coapSession_t *pSession, uint32_t dataLen)
\brief  This function is the callback function for CoAP factory reset message.

\param  [in]    sessionStatus   Status for CoAP session
\param  [in]    pData           Pointer to CoAP message payload
\param  [in]    pSession        Pointer to CoAP session
\param  [in]    dataLen         Length of CoAP payload
 ***************************************************************************************************/
static void APP_CoapResetToFactoryDefaultsCb
(
		coapSessionStatus_t sessionStatus,
		uint8_t *pData,
		coapSession_t *pSession,
		uint32_t dataLen
)
{
	THR_FactoryReset();
}
#endif

#if APP_AUTOSTART
/*!*************************************************************************************************
\private
\fn     static void APP_AutoStart(void)
\brief  This is the autostart function, used to start the network joining.

\param  [in]    param    Not used
 ***************************************************************************************************/
static void APP_AutoStart
(
		void *param
)
{
	if(!THR_GetAttr_IsDevConnected(mThrInstanceId))
	{
		mJoiningIsAppInitiated = TRUE;

		if(THR_NwkJoin(mThrInstanceId, THR_APP_JOIN_DISCOVERY_METHOD) != gThrStatus_Success_c)
		{
			/* User can treat join failure according to their application */
		}
	}
}

/*!*************************************************************************************************
\private
\fn     static void APP_AutoStartCb(void)
\brief  This is the autostart callback function.

\param  [in]    param    Not used
 ***************************************************************************************************/
static void APP_AutoStartCb
(
		void *param
)
{
	NWKU_SendMsg(APP_AutoStart, NULL, mpAppThreadMsgQueue);
}
#endif

/*==================================================================================================
Private debug functions
==================================================================================================*/
