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
#ifndef HUMIDITYSERVICE_H
#define HUMIDITYSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "profile_util.h"

/*******************************************************************************
 *                                CONSTANTS
 ******************************************************************************/
/* Characteristic Types - These must be listed in order that they appear
                          in service */
#define HUMIDITY_DATA             0x00
#define HUMIDITY_CONF             0x01
#define HUMIDITY_PERI             0x02

#define HUMIDITY_DATA_ID          PROFILE_ID_CREATE(HUMIDITY_DATA,PROFILE_VALUE)
#define HUMIDITY_CONF_ID          PROFILE_ID_CREATE(HUMIDITY_CONF,PROFILE_VALUE)
#define HUMIDITY_PERI_ID          PROFILE_ID_CREATE(HUMIDITY_PERI,PROFILE_VALUE)

/* Service UUID */
#define HUMIDITY_SERV_UUID        0xAA20
#define HUMIDITY_DATA_UUID        0xAA21
#define HUMIDITY_CONF_UUID        0xAA22
#define HUMIDITY_PERI_UUID        0xAA23

/* Length of sensor data in bytes */
#define HUMIDITY_DATA_LEN         4

/* Data readout periods (range 100 - 2550 ms) */
#define HUMIDITY_MIN_UPDATE_PERIOD 100 /* Minimum 100 milliseconds */
#define HUMIDITY_PERIOD_RESOLUTION 10  /* Resolution 10 milliseconds */

/* Length of sensor data in bytes */
#define HUMIDITY_DATA_DESCR       "Humidity Data (%)"
#define HUMIDITY_CONFIG_DESCR     "Humidity Conf."
#define HUMIDITY_PERIOD_DESCR     "Humidity Period"

/*******************************************************************************
 *                                API FUNCTIONS
 ******************************************************************************/

/*
 * Humidity_addService - Initializes the Sensor GATT Profile service
 *                      by registering GATT attributes with the GATT server.
 */
extern uint8_t Humidity_addService(void);

/*
 * Humidity_registerAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern uint8_t Humidity_registerAppCBs(BLEProfileCallbacks_t *appCallbacks);

/*
 * Humidity_setParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to write
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern uint8_t Humidity_setParameter(uint8_t param, uint8_t len,
        void *value);

/*
 * Humidity_getParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to read.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 */
extern uint8_t Humidity_getParameter(uint8_t param, void *value);

#ifdef __cplusplus
}
#endif

#endif /* HUMIDITYSERVICE_H */

