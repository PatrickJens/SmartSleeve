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
#include <pthread.h>
#include <unistd.h>
#include <ti/sail/bme280/bme280.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include "Profile/barometer_service.h"
#include "sensor_boosterpack_barometer.h"
#include "Board.h"
#include "sensor_configuration.h"

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t barometerTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Sensor Parameters */
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

/* Sensor Objects */
extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void *barometerTaskFxn(void *arg0);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
        uint8_t paramLen);

/*******************************************************************************
 *                                   FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      SensorBPBar_createTask
 *
 * @brief   Task creation function for barometer sensor
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void SensorBPBar_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = BAROMETER_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, BAROMETER_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&barometerTask, &pAttrs, barometerTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }
}

/*******************************************************************************
 * @fn      SensorBPBar_processCharChangeEvt
 *
 * @brief   SensorBP Barometer event handling
 *
 ******************************************************************************/
void SensorBPBar_processCharChangeEvt(uint8_t paramID)
{
    uint8_t newValue;

    switch (paramID)
    {
    case BAROMETER_CONF:
        if (sensorConfig != SENSORBSP_ERROR_DATA)
        {
            Barometer_getParameter(BAROMETER_CONF, &newValue);

            if (newValue == SENSORBSP_CFG_SENSOR_DISABLE)
            {
                /* Reset characteristics */
                initCharacteristicValue(BAROMETER_DATA, 0, BAROMETER_DATA_LEN);

                /* Deactivate task */
                pthread_mutex_lock(&lock);
                sampleData = false;
                pthread_mutex_unlock(&lock);

            } else
            {
                /* Activate task */
                pthread_mutex_lock(&lock);
                sampleData = true;
                pthread_cond_signal(&cond);
                pthread_mutex_unlock(&lock);
            }
            sensorConfig = newValue;
        } else
        {
            /* Make sure the previous characteristics value is restored */
            initCharacteristicValue(BAROMETER_CONF, sensorConfig, sizeof(uint8_t));
        }

        break;

    case BAROMETER_PERI:
        Barometer_getParameter(BAROMETER_PERI, &sensorPeriod);
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      barometerTaskInit
 *
 * @brief   Initialization function for the SensorTag barometer
 *
 ******************************************************************************/
static void barometerTaskInit(void)
{
    /* Initialize the module state variables */
    sensorConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    sensorPeriod = BAROMETER_DEFAULT_PERIOD;

    /* Initialize characteristics and sensor driver */
    sensorConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    initCharacteristicValue(BAROMETER_DATA, 0, BAROMETER_DATA_LEN);
    initCharacteristicValue(BAROMETER_CONF, SENSORBSP_CFG_SENSOR_DISABLE,
                            sizeof(uint8_t));
    Barometer_setParameter(BAROMETER_PERI, sizeof(uint16_t), &sensorPeriod);

    /* Initialize our mutex lock and condition */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

/*******************************************************************************
 * @fn      barometerTaskFxn
 *
 * @brief   The task loop of the humidity readout task
 *
 * @return  none
 ******************************************************************************/
static void *barometerTaskFxn(void *arg0)
{
    int32_t actualTemp = 0;
    uint32_t actualPress = 0;
    uint32_t actualHumidity = 0;
    uint32_t pauseTimeUS;

    /* Initialize the task */
    barometerTaskInit();

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

        /* Reading out the value */
        if(bme280_read_pressure_temperature_humidity(&actualPress,
                    &actualTemp, &actualHumidity) == BME280_INIT_VALUE)
        {
            /* Send data */
            Barometer_setParameter(BAROMETER_DATA, BAROMETER_DATA_LEN,
                                  &actualPress);
        }

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
    uint8_t data[BAROMETER_DATA_LEN];

    memset(data, value, paramLen);
    Barometer_setParameter(paramID, paramLen, data);
}
