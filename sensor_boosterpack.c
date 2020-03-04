/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
/*******************************************************************************
 *                              INCLUDES
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <mqueue.h>
#include <semaphore.h>
#include <time.h>
#include <unistd.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/sap/sap.h>
#include <ti/sbl/sbl.h>
#include <ti/sbl/sbl_image.h>

#include "Profile/profile_util.h"
#include "Profile/ir_temp_service.h"
#include "Profile/barometer_service.h"
#include "Profile/movement_service.h"
#include "Profile/humidity_service.h"
#include "Profile/optic_service.h"
#include "sensor_boosterpack_barometer.h"
#include "sensor_boosterpack_humidity.h"
#include "sensor_boosterpack_optical.h"
#include "sensor_boosterpack_movement.h"
#include "sensor_boosterpack_temperature.h"
#include "sensor_boosterpack.h"
#include "Board.h"
#include "platform.h"

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Used to block SNP calls during a synchronous transaction. */
static mqd_t sensQueueRec;
static mqd_t sensQueueSend;

/* Task configuration */
static pthread_t sensTask;

/* SAP Parameters for opening serial port to SNP */
static SAP_Params sapParams;

/* Device Name */
static uint8_t snpDeviceName[] =
        { 'M', 'S', 'P', '4', '3', '2', ' ', 'S', 'e', 'n', 's', 'o', 'r', 'H',
                'u', 'b' };

/* GAP - SCAN RSP data (max size = 31 bytes) */
static uint8_t scanRspData[] =
{
    /* Complete name */
    0x11, /* Length */
    SAP_GAP_ADTYPE_LOCAL_NAME_COMPLETE, 'M', 'S', 'P', '4', '3', '2', ' ', 'S',
    'e', 'n', 's', 'o', 'r', 'H', 'u', 'b',

    /* Connection interval range */
    0x05, /* Length */
    0x12,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    /* TX power level */
    0x02, /* Length */
    0x0A,
    0
};

/* GAP - Advertisement data (max size = 31 bytes, though this is
   best kept short to conserve power while advertising) */
static uint8_t advertData[] =
{
    /* Flags; this sets the device to use limited discoverable
       mode (advertises for 30 seconds at a time) instead of general
       discoverable mode (advertises indefinitely) */
    0x02, /* Length */
    SAP_GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | SAP_GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    /* Manufacturer specific advertising data */
    0x06, 0xFF,
    LO_UINT16(TI_COMPANY_ID),
    HI_UINT16(TI_COMPANY_ID),
    TI_ST_DEVICE_ID,
    TI_ST_KEY_DATA_ID, 0x00
};

/* Connection Handle - only one device currently allowed to connect to SNP */
static uint16_t connHandle = AP_DEFAULT_CONN_HANDLE;

/* BD Addr of the NWP */
static char nwpstr[] = "NWP:  0xFFFFFFFFFFFF";
#define nwpstrIDX       8

// BD Addr of peer device in connection
static char peerstr[] = "Peer: 0xFFFFFFFFFFFF";
#define peerstrIDX       8

/* Used for log messages */
extern Display_Handle displayOut;

/* Used for exiting threads */
extern pthread_t barometerTask;
extern pthread_t humidityTask;
extern pthread_t movementTask;
extern pthread_t opticalTask;
extern pthread_t temperatureTask;
extern pthread_t bmiSensorTask;
extern pthread_t sensorCommandTask;

/*******************************************************************************
 *                            FUNCTION PROTOTYPES
 ******************************************************************************/
static void AP_init(void);
static void *AP_taskFxn(void *arg0);
static void AP_initServices(void);
static void AP_keyHandler(void);
static void AP_bslKeyHandler(void);
static void AP_asyncCB(uint8_t cmd1, void *pParams);
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param);
static void AP_processIRTempChangeCB(uint8_t charID);
static void AP_processIRTempcccdCB(uint8_t charID, uint16_t value);
static void AP_processHumidityChangeCB(uint8_t charID);
static void AP_processHumiditycccdCB(uint8_t charID, uint16_t value);
static void AP_processBarometerChangeCB(uint8_t charID);
static void AP_processBarometercccdCB(uint8_t charID, uint16_t value);
static void AP_processOpticChangeCB(uint8_t charID);
static void AP_processOpticcccdCB(uint8_t charID, uint16_t value);
static void AP_processMovementChangeCB(uint8_t charID);
static void AP_processMovementcccdCB(uint8_t charID, uint16_t value);

/*******************************************************************************
 *                                 PROFILE CALLBACKS
 ******************************************************************************/
/*
 * Temperature Characteristic value change callback
 */
static BLEProfileCallbacks_t AP_IRTempSensorCBs =
{
    AP_processIRTempChangeCB,
    AP_processIRTempcccdCB
};

/*
 * Humidity Characteristic value change callback
 */
static BLEProfileCallbacks_t AP_HumiditySensorCBs =
{
    AP_processHumidityChangeCB,
    AP_processHumiditycccdCB
};

/*
 * Barometer Characteristic value change callback
 */
static BLEProfileCallbacks_t AP_BarometerSensorCBs =
{
    AP_processBarometerChangeCB,
    AP_processBarometercccdCB
};

/*
 * Optic Characteristic value change callback
 */
static BLEProfileCallbacks_t AP_OpticSensorCBs =
{
    AP_processOpticChangeCB,
    AP_processOpticcccdCB
};

/*
 * Movement Characteristic value change callback
 */
static BLEProfileCallbacks_t AP_MovementSensorCBs =
{
    AP_processMovementChangeCB,
    AP_processMovementcccdCB
};

/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn      AP_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
void AP_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = AP_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, AP_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&sensTask, &pAttrs, AP_taskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }
}

/*******************************************************************************
 * @fn      AP_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void AP_init(void)
{
    struct mq_attr attr;

    /* Create RTOS Queue */
    attr.mq_flags = 0;
    attr.mq_maxmsg = 64;
    attr.mq_msgsize = sizeof(uint32_t);
    attr.mq_curmsgs = 0;

    sensQueueRec = mq_open("SensorHub", O_RDWR | O_CREAT, 0664, &attr);
    sensQueueSend = mq_open("SensorHub", O_RDWR | O_CREAT | O_NONBLOCK, 0664,
                            &attr);

    /* Register Key Handler */
    GPIO_setCallback(Board_BUTTON0, (GPIO_CallbackFxn) AP_keyHandler);
    GPIO_enableInt (Board_BUTTON0);
    GPIO_setCallback(Board_BUTTON1, (GPIO_CallbackFxn) AP_bslKeyHandler);
    GPIO_enableInt (Board_BUTTON1);

    /* Write to the UART. */
    Display_print0(displayOut,0,0,
            "--------- Sensor Booster Pack Example ---------");
    Display_print0(displayOut,0,0,"Application Processor Initializing... ");

    /* Register to receive notifications from temperature if characteristics
       have been written to */
    IRTemp_registerAppCBs(&AP_IRTempSensorCBs);

    /* Register to receive notifications from Humidity Profile if characteristics
       have been written to */
    Humidity_registerAppCBs(&AP_HumiditySensorCBs);

    /* Register to receive notifications from Barometer Profile if characteristics
       have been written to */
    Barometer_registerAppCBs(&AP_BarometerSensorCBs);

    /* Register to receive notifications from Optic Profile if characteristics
       have been written to */
    Optic_registerAppCBs(&AP_OpticSensorCBs);

    /* Register to receive notifications from Movement Profile if characteristics
       have been written to */
    Movement_registerAppCBs(&AP_MovementSensorCBs);
}

/*******************************************************************************
 * @fn      AP_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 ******************************************************************************/
static void *AP_taskFxn(void *arg0)
{
    uint32_t apEvent = 0;
    struct timespec ts;
    ap_States_t state = AP_RESET;
    uint8_t enableAdv = 1;
    uint8_t disableAdv = 0;
    uint32_t prio = 0;
    uint8_t sblSatus;
    SBL_Params params;
    SBL_Image image;

    /* Initialize application */
    AP_init();

    Display_print0(displayOut,0,0,"Done!");

    while(1)
    {
        switch (state)
        {
        case AP_RESET:
        {
            /* Make sure CC26xx is not in BSL */
            GPIO_write(Board_RESET, Board_LED_OFF);
            GPIO_write(Board_MRDY, Board_LED_ON);

            usleep(10000);

            GPIO_write(Board_RESET, Board_LED_ON);

            /* Initialize UART port parameters within SAP parameters */
            SAP_initParams(SAP_PORT_REMOTE_UART, &sapParams);

            sapParams.port.remote.mrdyPinID = Board_MRDY;
            sapParams.port.remote.srdyPinID = Board_SRDY;
            sapParams.port.remote.boardID = Board_UART1;

            /* Setup NP module */
            SAP_open(&sapParams);

            /* Register Application thread's callback to receive
             * asynchronous requests from the NP.
             */
            SAP_setAsyncCB(AP_asyncCB);

            /* Reset the NP, and await a powerup indication.
               Clear any pending power indications received prior to this reset
               call */
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 1;

            mq_timedreceive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t), &prio,
                    &ts);

            SAP_reset();

            GPIO_clearInt(Board_BUTTON1);
            GPIO_enableInt(Board_BUTTON1);
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if ((apEvent != AP_EVT_BSL_BUTTON)
                        && (apEvent != AP_EVT_PUI))
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while ((apEvent != AP_EVT_BSL_BUTTON)
                    && (apEvent != AP_EVT_PUI));
            GPIO_disableInt(Board_BUTTON1);

            if (apEvent == AP_EVT_BSL_BUTTON)
            {
                state = AP_SBL;
            }
            else if (apEvent == AP_EVT_PUI)
            {
                /* Read BD ADDR */
                SAP_setParam(SAP_PARAM_HCI, SNP_HCI_OPCODE_READ_BDADDR, 0,
                                NULL);

                /* Setup Services - Service creation is blocking so
                 * no need to pend */
                AP_initServices();

                state = AP_IDLE;
            }

        }
            break;

        case AP_START_ADV:
        {
            /* Turn on user LED to indicate advertising */
            GPIO_write(Board_LED0, Board_LED_ON);
            Display_print0(displayOut,0,0, "Starting advertisement... ");

            /* Setting Advertising Name */
            SAP_setServiceParam(SNP_GGS_SERV_ID, SNP_GGS_DEVICE_NAME_ATT,
                                   sizeof(snpDeviceName), snpDeviceName);

            /* Set advertising data. */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_NOTCONN,
                    sizeof(advertData), advertData);

            /* Set Scan Response data. */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_DATA_SCANRSP,
                    sizeof(scanRspData), scanRspData);

            /* Enable Advertising and await NP response */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &enableAdv);
            
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_ADV_ENB)
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while (apEvent != AP_EVT_ADV_ENB);

            Display_print0(displayOut,0,0, "Done!");
            Display_print0(displayOut,0,0,
                    "Waiting for connection (or timeout)... ");

            /* Wait for connection or button press to cancel advertisement */
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_sec += 30;
            apEvent = 0;
            mq_timedreceive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                            &prio, &ts);

            if(apEvent == AP_EVT_CONN_EST)
            {
                state = AP_CONNECTED;
            }
            else
            {
                state = AP_CANCEL_ADV;
                Display_print0(displayOut, 0, 0,"Advertisement Timeout!");
            }
        }
            break;

        case AP_CONNECTED:
            /* Before connecting, NP will send the stop ADV message */
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_ADV_END)
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while (apEvent != AP_EVT_ADV_END);

            /* Update State and Characteristic values on LCD */
            Display_print1(displayOut,0,0,"Peer connected! (%s)", peerstr);

            /* Events that can happen during connection - Client Disconnection
                                                        - AP Disconnection */
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_CONN_TERM)
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while (apEvent != AP_EVT_CONN_TERM);

            /* Client has disconnected from server */
            SAP_setParam(SAP_PARAM_CONN, SAP_CONN_STATE, sizeof(connHandle),
                    (uint8_t *) &connHandle);

            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if ((apEvent != AP_EVT_CONN_TERM)
                        && (apEvent != AP_EVT_ADV_ENB))
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while ((apEvent != AP_EVT_CONN_TERM)
                    && (apEvent != AP_EVT_ADV_ENB));

            state = AP_CANCEL_ADV;

            break;

        case AP_CANCEL_ADV:
            Display_print0(displayOut,0,0,"Advertisement has been canceled!");

            /* Cancel Advertisement */
            SAP_setParam(SAP_PARAM_ADV, SAP_ADV_STATE, 1, &disableAdv);

            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if (apEvent != AP_EVT_ADV_END)
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while (apEvent != AP_EVT_ADV_END);

            state = AP_IDLE;
            break;

        case AP_IDLE:
            /* Turn off user LED to indicate stop advertising */
            GPIO_write(Board_LED0, Board_LED_OFF);

            Display_print0(displayOut,0,0,"State set to idle.");
            GPIO_clearInt(Board_BUTTON1);
            GPIO_enableInt(Board_BUTTON1);

            /* Key Press triggers state change from idle */
            do
            {
                apEvent = 0;
                mq_receive(sensQueueRec, (void*) &apEvent, sizeof(uint32_t),
                           &prio);

                if ((apEvent != AP_EVT_BUTTON_RIGHT)
                        && (apEvent != AP_EVT_BSL_BUTTON))
                {
                    Display_printf(displayOut, 0, 0,
                                   "[bleThread] Warning! Unexpected Event %lu",
                                   apEvent);
                }
            }
            while ((apEvent != AP_EVT_BUTTON_RIGHT)
                    && (apEvent != AP_EVT_BSL_BUTTON));
            GPIO_disableInt(Board_BUTTON1);

            if (apEvent == AP_EVT_BUTTON_RIGHT)
            {
                state = AP_START_ADV;
            }
            else if (apEvent == AP_EVT_BSL_BUTTON)
            {
                state = AP_SBL;
            }

            break;
        case AP_SBL:
        {
            Display_print0(displayOut,0,0,"Device being set into BSL mode... ");
            SAP_close();
            
            /* Stopping all of the other running threads (since we are
                resetting this doesn't have to be clean) */
            pthread_cancel(barometerTask);
            pthread_cancel(humidityTask);
            pthread_cancel(movementTask);
            pthread_cancel(opticalTask);
            pthread_cancel(temperatureTask);
            pthread_cancel(bmiSensorTask);
            pthread_cancel(sensorCommandTask);

            /* Initialize SBL Params and open port to target device */
            SBL_initParams(&params);
            params.resetPinID = Board_RESET;
            params.blPinID = Board_MRDY;
            params.targetInterface = SBL_DEV_INTERFACE_UART;
            params.localInterfaceID = Board_UART1;
            SBL_open(&params);

            /* Reset target and force into SBL code */
            SBL_openTarget();

            Display_print0(displayOut,0,0,"Done!");
            Display_print0(displayOut,0,0,"Programming the CC26xx... ");

            /* Flash new image to target */
            image.imgType = SBL_IMAGE_TYPE_INT;
            image.imgInfoLocAddr = (uint32_t)&SNP_code[0];
            image.imgLocAddr = (uint32_t)&SNP_code[0];
            image.imgTargetAddr = SNP_IMAGE_START;
            sblSatus = SBL_writeImage(&image);

            if (sblSatus != SBL_SUCCESS)
            {
                Display_print0(displayOut,0,0,"Programming failed!");

            } else
            {
                Display_print0(displayOut,0,0,"Programming passed!");
            }

            Display_print0(displayOut,0,0,"Resetting device.");

            /* Reset target and exit SBL code */
            SBL_closeTarget();

            /* Close SBL port to target device */
            SBL_close();

            sleep(1);
            MCU_rebootDevice();
        }
        break;

        default:
            break;
        }
    }
}

/*******************************************************************************
 * @fn      AP_initServices
 *
 * @brief   Configure SNP and register services.
 *
 * @param   None.
 *
 * @return  None.
 ******************************************************************************/
static void AP_initServices(void)
{
    IRTemp_addService();
    Humidity_addService();
    Barometer_addService();
    Optic_addService();
    Movement_addService();

    SAP_registerEventCB(AP_processSNPEventCB, 0xFFFF);
}

/*
 * This is a callback operating in the NPI task.
 * These are events this application has registered for.
 *
 */
static void AP_processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
    uint32_t eventPend;

    switch (event)
    {
    case SNP_CONN_EST_EVT:
    {
        snpConnEstEvt_t * connEstEvt = (snpConnEstEvt_t *) param;

        /* Update Peer Addr String */
        connHandle = connEstEvt->connHandle;
        ProfileUtil_convertBdAddr2Str(&peerstr[peerstrIDX], connEstEvt->pAddr);

        /* Notify state machine of established connection */
        eventPend = AP_EVT_CONN_EST;
        mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
    }
        break;

    case SNP_CONN_TERM_EVT:
    {
        connHandle = AP_DEFAULT_CONN_HANDLE;
        eventPend = AP_EVT_CONN_TERM;
        mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);

    }
        break;

    case SNP_ADV_STARTED_EVT:
    {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *) param;
        if (advEvt->status == SNP_SUCCESS)
        {
            /* Notify state machine of Advertisement Enabled */
            eventPend = AP_EVT_ADV_ENB;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
        }
        else
        {
            eventPend = AP_ERROR;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
        }
    }
        break;

    case SNP_ADV_ENDED_EVT:
    {
        snpAdvStatusEvt_t *advEvt = (snpAdvStatusEvt_t *) param;
        if (advEvt->status == SNP_SUCCESS)
        {
            /* Notify state machine of Advertisement Disabled */
            eventPend = AP_EVT_ADV_END;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
        }
    }
        break;

    default:
        break;
    }
}

/*
 * This is a callback operating in the NPI task.
 * These are Asynchronous indications.
  */
static void AP_asyncCB(uint8_t cmd1, void *pParams)
{
    uint32_t eventPend;

    switch (SNP_GET_OPCODE_HDR_CMD1(cmd1))
    {
    case SNP_DEVICE_GRP:
    {
        switch (cmd1)
        {
        case SNP_POWER_UP_IND:
        {
            eventPend = AP_EVT_PUI;
            mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
            break;
        }
        case SNP_HCI_CMD_RSP:
        {
            snpHciCmdRsp_t *hciRsp = (snpHciCmdRsp_t *) pParams;
            switch (hciRsp->opcode)
            {
            case SNP_HCI_OPCODE_READ_BDADDR:
                ProfileUtil_convertBdAddr2Str(&nwpstr[nwpstrIDX],
                        hciRsp->pData);
              default:
                break;
            }
        }
            break;

        case SNP_EVENT_IND:
        {
        }
        default:
            break;
        }
    }
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      AP_bslKeyHandler
 *
 * @brief   event handler function to notify the app to program the SNP
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void AP_bslKeyHandler(void)
{
    uint32_t delayDebounce = 0;
    uint32_t eventPend;

    GPIO_disableInt(Board_BUTTON1);

    /* Delay for switch debounce */
    for (delayDebounce = 0; delayDebounce < 20000; delayDebounce++);

    GPIO_clearInt(Board_BUTTON1);
    GPIO_enableInt(Board_BUTTON1);

    GPIO_toggle (Board_LED1);

    eventPend = AP_EVT_BSL_BUTTON;
    mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
}

/*******************************************************************************
 * @fn      AP_keyHandler
 *
 * @brief   Key event handler function
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void AP_keyHandler(void)
{
    uint32_t delayDebounce = 0;
    uint32_t eventPend;

    GPIO_disableInt (Board_BUTTON0);

    /* Delay for switch debounce */
    for (delayDebounce = 0; delayDebounce < 20000; delayDebounce++)
        ;

    GPIO_clearInt(Board_BUTTON0);
    GPIO_enableInt(Board_BUTTON0);

    eventPend = AP_EVT_BUTTON_RIGHT;
    mq_send(sensQueueSend, (void*)&eventPend, sizeof(uint32_t), 1);
}

/*
 * Callbacks for the Tmp Sensor
 */
static void AP_processIRTempChangeCB(uint8_t charID)
{
    SensorBPTmp_processCharChangeEvt(charID);
}

static void AP_processIRTempcccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case IRTEMP_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set start periodic
               clock that will write to characteristic 4 */
            if (value
                & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                Display_print0(displayOut,0,0,
                        "Temperature update enabled!");
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/*
 * Callbacks for the Humidity Sensor
 */
static void AP_processHumidityChangeCB(uint8_t charID)
{
    SensorBPHum_processCharChangeEvt(charID);
}

static void AP_processHumiditycccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case HUMIDITY_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set */
            if (value
                & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                Display_print0(displayOut,0,0,"Humidity update enabled!");
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/*
 * Callbacks for the Barometer Sensor
 */
static void AP_processBarometerChangeCB(uint8_t charID)
{
    SensorBPBar_processCharChangeEvt(charID);
}

static void AP_processBarometercccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case BAROMETER_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set */
            if (value
                & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                Display_print0(displayOut, 0, 0, "Barometer update enabled!");
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/*
 * Callbacks for the Optic Sensor
 */
static void AP_processOpticChangeCB(uint8_t charID)
{
    SensorBPOpt_processCharChangeEvt(charID);
}

static void AP_processOpticcccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case OPTIC_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set */
            if (value
                  & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                Display_print0(displayOut, 0, 0, "Optic update enabled!");
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}

/*
 * Callbacks for the Movement Sensor
 */
static void AP_processMovementChangeCB(uint8_t charID)
{
    SensorBPMov_processCharChangeEvt(charID);
}

static void AP_processMovementcccdCB(uint8_t charID, uint16_t value)
{
    switch (PROFILE_ID_CHAR(charID))
    {
    case OPTIC_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* If indication or notification flags are set */
            if (value
                  & (SNP_GATT_CLIENT_CFG_NOTIFY | SNP_GATT_CLIENT_CFG_INDICATE))
            {
                Display_print0(displayOut,0,0,"Movement update enabled!");
            }
            break;

        default:
            break;
        }
        break;

    default:
        break;
    }
}
