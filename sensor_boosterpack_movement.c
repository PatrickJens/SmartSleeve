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
#include <semaphore.h>
#include <mqueue.h>
#include <pthread.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>
#include <ti/sail/bmi160/bmi160.h>
#include "Profile/movement_service.h"
#include "sensor_boosterpack_movement.h"
#include "sensor_configuration.h"
#include "bmi160_support.h"
#include "Board.h"
#include <stdio.h>

/*******************************************************************************
 *                             VARIABLES
 ******************************************************************************/
/* Task setup */
pthread_t movementTask;
pthread_t bmiSensorTask;
pthread_t sensorCommandTask;
static pthread_mutex_t lock;
static pthread_cond_t cond;
static volatile bool sampleData;

/* Sensor Parameters */
static uint8_t sensorConfig;
static uint16_t sensorPeriod;

/* Sensor Objects */
extern I2C_Handle i2cHandle;
extern Display_Handle displayOut;
extern struct bmi160_t s_bmi160;
struct bmi160_fifo_data_header_t header_data;
static BMI160_RETURN_FUNCTION_TYPE com_rslt;
static sem_t bmi160Sem;
static sem_t publishDataSem;
static struct bmi160_gyro_t gyroxyz_mov = { 0, 0, 0 };
static struct bmi160_accel_t accelxyz_mov = { 0, 0, 0 };
static struct bmi160_mag_xyz_s32_t magxyz_mov = { 0, 0, 0 };
static mqd_t bmiCommandRec;
static mqd_t bmiCommandSend;

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static int8_t BMI160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *reg_data, uint8_t cnt);
static int8_t BMI160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *reg_data, uint8_t cnt);
static int8_t BMI160_I2C_burst_read(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t *reg_data, uint32_t cnt);
static void BMI160_delay_msek(uint32_t msek);
static bool i2cTransferFxn(I2C_Handle handle, I2C_Transaction transaction);
static void *movementTaskFxn(void *arg0);
static void *bmiInterruptHandlerTask(void *arg0);
static void *bmiCommandFunction(void *arg0);
static void initCharacteristicValue(uint8_t paramID, uint8_t value,
        uint8_t paramLen);
static void bmi160Callback(uint_least8_t index);

/*******************************************************************************
 *                                   FUNCTIONS
 ******************************************************************************/

/*******************************************************************************
 * @fn      SensorBPMov_createTask
 *
 * @brief   Task creation function for the movement task
 *
 * @param   none
 *
 * @return  none
 ******************************************************************************/
void SensorBPMov_createTask(void)
{
    pthread_attr_t pAttrs;
    struct sched_param priParam;
    int retc;
    int detachState;
    struct mq_attr attr;

    pthread_attr_init(&pAttrs);
    priParam.sched_priority = MOVEMENT_TASK_PRIORITY;

    detachState = PTHREAD_CREATE_DETACHED;
    retc = pthread_attr_setdetachstate(&pAttrs, detachState);

    if (retc != 0)
    {
        while(1);
    }

    pthread_attr_setschedparam(&pAttrs, &priParam);

    retc |= pthread_attr_setstacksize(&pAttrs, MOVEMENT_TASK_STACK_SIZE);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&movementTask, &pAttrs, movementTaskFxn, NULL);

    if (retc != 0)
    {
        while(1);
    }

    retc = pthread_create(&bmiSensorTask, &pAttrs, bmiInterruptHandlerTask,
                          NULL);

    if (retc != 0)
    {
        while (1);
    }

    retc = pthread_create(&sensorCommandTask, &pAttrs, bmiCommandFunction,
                          NULL);

    if (retc != 0)
    {
        while (1);
    }

    /* Initializing the semaphore */
    sem_init(&bmi160Sem,0,0);
    sem_init(&publishDataSem,0,0);

    /* Initializing the message queues */
    attr.mq_flags = 0;
    attr.mq_maxmsg = 64;
    attr.mq_msgsize = sizeof(uint32_t);
    attr.mq_curmsgs = 0;

    bmiCommandRec = mq_open("bmi160", O_RDWR | O_CREAT, 0664, &attr);
    bmiCommandSend = mq_open("bmi160", O_RDWR | O_CREAT | O_NONBLOCK, 0664,
                            &attr);
}

/*******************************************************************************
 * @fn      SensorBPMov_processCharChangeEvt
 *
 * @brief   SensorTag Movement event handling
 *
 ******************************************************************************/
void SensorBPMov_processCharChangeEvt(uint8_t paramID)
{
    uint16_t newValue;
    uint32_t sendCmd;

    switch (paramID)
    {
    case MOVEMENT_CONF:
        if (sensorConfig != SENSORBSP_ERROR_DATA)
        {
            Movement_getParameter(MOVEMENT_CONF, &newValue);

            if (newValue == SENSORBSP_CFG_SENSOR_DISABLE)
            {
                /* Reset characteristics */
                initCharacteristicValue(MOVEMENT_DATA, 0, MOVEMENT_DATA_LEN);

                /* Deactivate task */
                sendCmd = MOVEMENT_SENSOR_STOP;
                mq_send(bmiCommandSend, (void*)&sendCmd, sizeof(uint32_t), 1);
                pthread_mutex_lock(&lock);
                sampleData = false;
                pthread_mutex_unlock(&lock);
            }
            else
            {
                /* Activate task */
                sendCmd = MOVEMENT_SENSOR_START;
                mq_send(bmiCommandSend, (void*)&sendCmd, sizeof(uint32_t), 1);
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
            initCharacteristicValue(MOVEMENT_CONF, sensorConfig,
                                    sizeof(uint8_t));
        }

        break;

    case MOVEMENT_PERI:
        Movement_getParameter(MOVEMENT_PERI, &sensorPeriod);
        break;

    default:
        break;
    }
}

/*******************************************************************************
 * @fn      movementTaskInit
 *
 * @brief   Initialization function for the SensorTag humidity sensor
 *
 ******************************************************************************/
static void movementTaskInit(void)
{
    /* Initialize characteristics and sensor driver */
    sensorConfig = SENSORBSP_CFG_SENSOR_DISABLE;
    sensorPeriod = MOVEMENT_DEFAULT_PERIOD;
    initCharacteristicValue(MOVEMENT_DATA, 0, MOVEMENT_DATA_LEN);
    initCharacteristicValue(MOVEMENT_CONF,
                            SENSORBSP_CFG_SENSOR_DISABLE,
                            sizeof(uint8_t));
    Movement_setParameter(MOVEMENT_PERI, sizeof(uint16_t), &sensorPeriod);

    /* Initializing the sensor */
    com_rslt = BMI160_INIT_VALUE;
    s_bmi160.bus_write = BMI160_I2C_bus_write;
    s_bmi160.bus_read = BMI160_I2C_bus_read;
    s_bmi160.burst_read = BMI160_I2C_burst_read;
    s_bmi160.delay_msec = BMI160_delay_msek;
    s_bmi160.dev_addr = BMI160_I2C_ADDR2;
    com_rslt += bmi160_init(&s_bmi160);

    /* Setting GPIO Callbacks */
    GPIO_setCallback(Board_SensorBP_INT1, &bmi160Callback);
    GPIO_clearInt(Board_SensorBP_INT1);
    GPIO_enableInt(Board_SensorBP_INT1);

    /* Initializing the mutex */
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
}

/*******************************************************************************
 * @fn      movementTaskFxn
 *
 * @brief   The task loop of the movement readout task
 *
 * @return  none
 ******************************************************************************/
static void *movementTaskFxn(void *arg0)
{
    uint8_t sensorData[MOVEMENT_DATA_LEN];
    uint32_t pauseTimeUS;

    /* Initialize the task */
    movementTaskInit();

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

        /* Waiting for data to become available from the sensor */
        sem_wait(&publishDataSem);

        /* Populating and publishing data */
        memcpy(sensorData, &accelxyz_mov, 6);
        memcpy(sensorData + 6, &gyroxyz_mov, 6);
        memcpy(sensorData + 12, &magxyz_mov, 6);

        Movement_setParameter(MOVEMENT_DATA, MOVEMENT_DATA_LEN, sensorData);

        pauseTimeUS = sensorPeriod * 500; //pauseTimeUS = sensorPeriod * 1000;

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
 * @fn      bmiInterruptHandlerTask
 *
 * @brief   The task loop of the sensor data processing task. This task is
 *          use to process the data from the BMI160's FIFO
 *
 * @return  none
 ******************************************************************************/
void* bmiInterruptHandlerTask(void *arg0)
{
    uint32_t ii;

    while (1)
    {
        sem_wait(&bmi160Sem);

        com_rslt += bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
                                                 &header_data);

        com_rslt += bmi160_read_fifo_header_data(BMI160_SEC_IF_BMM150,
                                                 &header_data);
        for (ii = 0; ii <= header_data.accel_frame_count; ii++)
        {
            if (ii == 0)
            {
                accelxyz_mov.x = header_data.accel_fifo[ii].x;
                accelxyz_mov.y = header_data.accel_fifo[ii].y;
                accelxyz_mov.z = header_data.accel_fifo[ii].z;
            }
            else
            {
                accelxyz_mov.x = (accelxyz_mov.x + header_data.accel_fifo[ii].x)
                        / 2;
                accelxyz_mov.y = (accelxyz_mov.y + header_data.accel_fifo[ii].y)
                        / 2;
                accelxyz_mov.z = (accelxyz_mov.z + header_data.accel_fifo[ii].z)
                        / 2;
            }
        }
        for (ii = 0; ii <= header_data.gyro_frame_count; ii++)
        {
            if (ii == 0)
            {
                gyroxyz_mov.x = header_data.gyro_fifo[ii].x;
                gyroxyz_mov.y = header_data.gyro_fifo[ii].y;
                gyroxyz_mov.z = header_data.gyro_fifo[ii].z;
            }
            else
            {
                gyroxyz_mov.x = (gyroxyz_mov.x + header_data.gyro_fifo[ii].x)
                        / 2;
                gyroxyz_mov.y = (gyroxyz_mov.y + header_data.gyro_fifo[ii].y)
                        / 2;
                gyroxyz_mov.z = (gyroxyz_mov.z + header_data.gyro_fifo[ii].z)
                        / 2;
            }
        }
        for (ii = 0; ii <= header_data.mag_frame_count; ii++)
        {
            if (ii == 0)
            {
                magxyz_mov.x = header_data.mag_fifo[ii].x;
                magxyz_mov.y = header_data.mag_fifo[ii].y;
                magxyz_mov.z = header_data.mag_fifo[ii].z;
            }
            else
            {
                magxyz_mov.x = (magxyz_mov.x + header_data.mag_fifo[ii].x) / 2;
                magxyz_mov.y = (magxyz_mov.y + header_data.mag_fifo[ii].y) / 2;
                magxyz_mov.z = (magxyz_mov.z + header_data.mag_fifo[ii].z) / 2;
            }
        }

        sem_post(&publishDataSem);
    }
}

/*******************************************************************************
 * @fn      bmiCommandFunction
 *
 * @brief   Handles enabling/disabling the BMI sensor. Since these commands
 *          are given over BLE we have to have a non-ISR context thread to
 *          handle the commands and send them over I2C.
 *
 * @return  none
 ******************************************************************************/
static void *bmiCommandFunction(void *arg0)
{
    uint32_t prio = 0;
    uint32_t curCommand = 0;

    while(1)
    {
        mq_receive(bmiCommandRec, (void*) &curCommand, sizeof(uint32_t), &prio);

        if(curCommand == MOVEMENT_SENSOR_START)
        {
            /* Reset the bmi160  interrupt engine, FIFO */
            bmi160_set_command_register(0xB0);
            bmi160_set_command_register(0xB1);
            com_rslt += bmi160_config_running_mode(STANDARD_UI_9DOF_FIFO);
        }
        else if(curCommand == MOVEMENT_SENSOR_STOP)
        {
            com_rslt += bmi160_set_command_register(GYRO_MODE_SUSPEND);
            com_rslt += bmi160_set_command_register(MAG_MODE_SUSPEND);
            com_rslt += bmi160_set_command_register(ACCEL_SUSPEND);
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
    uint8_t data[MOVEMENT_DATA_LEN];

    memset(data, value, paramLen);
    Movement_setParameter(paramID, paramLen, data);
}

/*******************************************************************************
 * @fn      bmi160Callback
 *
 * @brief   Callback value that signifies a BMI160 reading has happened
 *
 * @param   index - Index of BMI160 Sensor
 *
 * @return  none
 ******************************************************************************/
static void bmi160Callback(uint_least8_t index)
{
    sem_post(&bmi160Sem);
}

/*
 * Sensor interface functions. These functions are used to interact with the
 * SAIL BMI160 module and use for generic I2C write/reads. You shouldn't have
 * to edit these.
 */
static int8_t BMI160_I2C_bus_write(uint8_t dev_addr, uint8_t reg_addr,
                            uint8_t *reg_data, uint8_t cnt)
{
    int32_t ierror = BMI160_INIT_VALUE;
    uint8_t array[I2C_BUFFER_LEN];
    uint8_t stringpos = BMI160_INIT_VALUE;
    I2C_Transaction i2cTransaction;

    array[BMI160_INIT_VALUE] = reg_addr;
    for (stringpos = BMI160_INIT_VALUE; stringpos < cnt; stringpos++)
        array[stringpos + BMI160_GEN_READ_WRITE_DATA_LENGTH] = *(reg_data
                + stringpos);

    i2cTransaction.writeBuf = array;
    i2cTransaction.writeCount = cnt + 1;
    i2cTransaction.readCount = 0;
    i2cTransaction.slaveAddress = dev_addr;

    /* If transaction success */
    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        ierror = -1;
    }

    return (int8_t) ierror;
}

static int8_t BMI160_I2C_bus_read(uint8_t dev_addr, uint8_t reg_addr,
                           uint8_t *reg_data, uint8_t cnt)
{
    int32_t ierror = BMI160_INIT_VALUE;
    I2C_Transaction i2cTransaction;

    i2cTransaction.writeBuf = &reg_addr;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = cnt;
    i2cTransaction.slaveAddress = dev_addr;

    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        ierror = -1;
    }

    return (int8_t) ierror;
}

static int8_t BMI160_I2C_burst_read(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t *reg_data, uint32_t cnt)
{
    int32_t ierror = BMI160_INIT_VALUE;

    I2C_Transaction i2cTransaction;
    i2cTransaction.writeBuf = &reg_addr;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = reg_data;
    i2cTransaction.readCount = cnt;
    i2cTransaction.slaveAddress = dev_addr;

    if (!i2cTransferFxn(i2cHandle, i2cTransaction))
    {
        ierror = -1;
    }

    return (int8_t) ierror;
}

static void BMI160_delay_msek(uint32_t msek)
{
    usleep(msek * 1000);
}

static bool i2cTransferFxn(I2C_Handle handle, I2C_Transaction transaction)
{
    if (I2C_transfer(handle, &transaction))
    {
        return (true);
    }
    else
    {
        return (false);
    }
}
