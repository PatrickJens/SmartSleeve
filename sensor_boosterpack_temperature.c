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
/* External Defines */
#include <ti/sail/tmp007/tmp007.h>
#include <ti/display/Display.h>
#include <ti/drivers/I2C.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Local Defines */
#include "Profile/ir_temp_service.h"
#include "sensor_boosterpack_temperature.h"
#include "Board.h"
#include "sensor_configuration.h"

/*******************************************************************************
 *                             LOCAL VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t temperatureTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Sensor Parameters */
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

/* Sensor Objects */
extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;
static TMP007_Handle  tmp007Handle = NULL;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *temperatureTaskFxn(void *arg0);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
        uint8_t paramLen);

/*******************************************************************************
 *                                    FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn      SensorBPTmp_createTask
 *
 * @brief   Task creation function for the SensorBP
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void SensorBPTmp_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = IRTEMP_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, IRTEMP_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&temperatureTask, &pAttrs, temperatureTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }
}

/*******************************************************************************
 * @fn      SensorBPTmp_processCharChangeEvt
 *
 * @brief   SensorBP IR temperature event handling
 *
 ******************************************************************************/
void SensorBPTmp_processCharChangeEvt(uint8_t paramID)
{
    uint8_t newValue;

    switch (paramID)
    {
    case IRTEMP_CONF:
        if (sensorConfig != SENSORBSP_ERROR_DATA)
        {
            IRTemp_getParameter(IRTEMP_CONF, &newValue);

            if (newValue == SENSORBSP_CFG_SENSOR_DISABLE)
            {
                /* Reset characteristics */
                initCharacteristicValue(IRTEMP_DATA, 0, IRTEMP_DATA_LEN);

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
            initCharacteristicValue(IRTEMP_CONF, sensorConfig, sizeof(uint8_t));
        }

        /* Make sure sensor is disabled */
        break;

    case IRTEMP_PERI:
        IRTemp_getParameter(IRTEMP_PERI, &sensorPeriod);
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      temperatureTaskInit
 *
 * @brief   Initialization function for the SensorBP IR temperature sensor
 *
 ******************************************************************************/
static void temperatureTaskInit(void)
{
    TMP007_Params  tmp007Params;

    /* Initialize the module state variables */
    sensorPeriod = IRTEMP_DEFAULT_PERIOD;

    /* Initialize characteristics and sensor driver */
    sensorConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    initCharacteristicValue(IRTEMP_DATA, 0, IRTEMP_DATA_LEN);
    initCharacteristicValue(IRTEMP_CONF, sensorConfig, sizeof(uint8_t));
    IRTemp_setParameter(IRTEMP_PERI, sizeof(uint16_t), &sensorPeriod);

    /* Initialize tmp007Params structure to defaults */
    TMP007_Params_init(&tmp007Params);

    /* Set the conversions, affects conversion time */
    tmp007Params.conversions = TMP007_4CONV;

    /* Open TMP007 sensor with custom Params */
    tmp007Handle = TMP007_open(Board_TMP007, i2cHandle, &tmp007Params);

    /* Initialize our mutex lock and condition */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

/*******************************************************************************
 * @fn      temperatureTaskFxn
 *
 * @brief   The task loop of the temperature readout task
 *
 * @return  none
 ******************************************************************************/
static void *temperatureTaskFxn(void *arg0)
{
    float temp;
    uint32_t pauseTimeUS;

    /* Initialize the task */
    temperatureTaskInit();

    /* Deactivate task (active only when measurement is enabled) */
    sampleData = false;

    while(1)
    {
        pthread_mutex_lock(&lock);
        while(!sampleData)
        {
            pthread_cond_wait(&cond, &lock);
        }
        pthread_mutex_unlock(&lock);

        if (!TMP007_getObjTemp(tmp007Handle, TMP007_CELSIUS, &temp))
        {
            Display_print0(displayOut, 0, 0, "TMP007 sensor read failed");

        }

        /* Update data */
        IRTemp_setParameter(IRTEMP_DATA, IRTEMP_DATA_LEN, &temp);

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
    uint8_t data[IRTEMP_DATA_LEN];

    memset(data, value, paramLen);
    IRTemp_setParameter(paramID, paramLen, data);
}
