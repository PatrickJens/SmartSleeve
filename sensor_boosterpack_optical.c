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
 *                                INCLUDES
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/sail/opt3001/opt3001.h>
#include "Profile/optic_service.h"
#include "sensor_boosterpack_optical.h"
#include "sensor_configuration.h"
#include "Board.h"

/*******************************************************************************
 *                             LOCAL VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t opticalTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Parameters */
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

/* Sensor Objects */
extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;
static OPT3001_Handle opt3001Handle = NULL;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *opticalTaskFxn(void *arg0);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
        uint8_t paramLen);

/*******************************************************************************
 *                                    FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn      SensorBPOpt_createTask
 *
 * @brief   Task creation function for the SensorBP
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void SensorBPOpt_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = OPTIC_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, OPTIC_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&opticalTask, &pAttrs, opticalTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }
}

/*******************************************************************************
 * @fn      SensorBPOpt_processCharChangeEvt
 *
 * @brief   SensorBP Optic event handling
 *
 ******************************************************************************/
void SensorBPOpt_processCharChangeEvt(uint8_t paramID)
{
    uint8_t newValue;

    switch (paramID)
    {
    case OPTIC_CONF:
        if (sensorConfig != SENSORBSP_ERROR_DATA)
        {
            Optic_getParameter(OPTIC_CONF, &newValue);

            if (newValue == SENSORBSP_CFG_SENSOR_DISABLE)
            {
                /* Reset characteristics */
                initCharacteristicValue(OPTIC_DATA, 0, OPTIC_DATA_LEN);

                /* Deactivate task */
                pthread_mutex_lock(&lock);
                sampleData = false;
                pthread_mutex_unlock(&lock);
            }
            else
            {
                /* Activate task */
                pthread_mutex_lock(&lock);
                sampleData = true;
                pthread_cond_signal(&cond);
                pthread_mutex_unlock(&lock);
            }

            sensorConfig = newValue;
        }
        else
        {
            /* Make sure the previous characteristics value is restored */
            initCharacteristicValue(OPTIC_CONF, sensorConfig, sizeof(uint8_t));
        }
        break;

    case OPTIC_PERI:
        Optic_getParameter(OPTIC_PERI, &sensorPeriod);
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      opticalTaskInit
 *
 * @brief   Initialization function for the SensorBP IR temperature sensor
 *
 ******************************************************************************/
static void opticalTaskInit(void)
{
    OPT3001_Params opt3001Params;

    /* Initialize characteristics and sensor driver */
    sensorConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    sensorPeriod = OPTIC_DEFAULT_PERIOD;
    initCharacteristicValue(OPTIC_DATA, 0, OPTIC_DATA_LEN);
    initCharacteristicValue(OPTIC_CONF, sensorConfig, sizeof(uint8_t));
    Optic_setParameter(OPTIC_PERI, sizeof(uint16_t), &sensorPeriod);

    /* Initialize opt3001Params structure to defaults */
    OPT3001_Params_init(&opt3001Params);

    /* Open OPT3001 sensor with custom parameters */
    opt3001Handle = OPT3001_open(Board_OPT3001, i2cHandle,
            &opt3001Params);

    /* Check if the open is successful */
    if (opt3001Handle == NULL)
    {
        Display_print0(displayOut, 0, 0, "OPT3001 Open Failed!");
        while (1);
    }

    /* Initializing the mutex */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

/*******************************************************************************
 * @fn      opticalTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @return  none
 ******************************************************************************/
static void *opticalTaskFxn(void *arg0)
{
    float luxValue;
    uint32_t pauseTimeUS;

    /* Initialize the task */
    opticalTaskInit();

    /* Deactivate task (active only when measurement is enabled) */
    sampleData = false;

    while (1)
    {
        pthread_mutex_lock(&lock);
        while(!sampleData)
        {
            pthread_cond_wait(&cond, &lock);
        }
        pthread_mutex_unlock(&lock);

        /* Get current Lux */
        if (!OPT3001_getLux(opt3001Handle, &luxValue))
        {
            Display_print0(displayOut, 0, 0, "OPT3001 sensor read failed");
        }

        /* Convert/Send data */
        Optic_setParameter(OPTIC_DATA, OPTIC_DATA_LEN, &luxValue);

        pauseTimeUS = sensorPeriod * 1000;

        /* Next cycle */
        if(pauseTimeUS >= 1000000)
        {
            sleep(pauseTimeUS/1000000);
        }
        else
        {
            usleep(pauseTimeUS);
        }
    }
}

/*******************************************************************************
 * @fn      initCharacteristicValue
 *
 * @brief   Initialize a characteristic value
 *
 * @param   paramID - parameter ID of the value is to be cleared
 *
 * @param   value - value to initialize with
 *
 * @param   paramLen - length of the parameter
 *
 * @return  none
 ******************************************************************************/
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
        uint8_t paramLen)
{
    uint8_t data[OPTIC_DATA_LEN];

    memset(data, value, paramLen);
    Optic_setParameter(paramID, paramLen, data);
}
