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
#ifndef SENSOR_BOOSTERPACK_BAROMETER_H_
#define SENSOR_BOOSTERPACK_BAROMETER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                                CONSTANTS
 ******************************************************************************/
/* How often to perform sensor reads (milliseconds) */
#define BAROMETER_DEFAULT_PERIOD   1000

/* Constants for two-stage reading */
#define BAROMETER_FSM_PERIOD       80

/* Task configuration */
#define BAROMETER_TASK_PRIORITY    1
#define BAROMETER_TASK_STACK_SIZE  1024


/*******************************************************************************
 *                                FUNCTIONS
 ******************************************************************************/
/*
 * Create the Barometer sensor task
 */
void SensorBPBar_createTask(void);

/*
 * Task Event Processor for characteristic changes
 */
extern void SensorBPBar_processCharChangeEvt(uint8_t paramID);

/*
 * Task Event Processor for the BLE Application
 */
extern void SensorBPBar_reset(void);



#ifdef __cplusplus
}
#endif

#endif /* SENSOR_BOOSTERPACK_BAROMETER_H_ */
