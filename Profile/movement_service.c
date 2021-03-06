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
#include <ti/sap/snp.h>
#include <ti/sap/snp_rpc.h>
#include <ti/sap/snp_rpc_synchro.h>
#include <ti/sap/sap.h>
#include "movement_service.h"
#include "profile_util.h"

/*******************************************************************************
 *                                   MACROS
 ******************************************************************************/
#define MOVEMENT_NUM_ATTR_SUPPORTED 3

/*******************************************************************************
 *                             LOCAL VARIABLES
 ******************************************************************************/

/* Service UUID */
static uint8_t sensorServiceUUID[SNP_128BIT_UUID_SIZE] =
{
    TI_BASE_UUID_128(MOVEMENT_SERV_UUID)
};

/* Characteristic UUID: data */
static uint8_t sensorDataUUID[SNP_128BIT_UUID_SIZE] =
{
    TI_BASE_UUID_128(MOVEMENT_DATA_UUID),
};

/* Characteristic UUID: config */
static uint8_t sensorCfgUUID[SNP_128BIT_UUID_SIZE] =
{
    TI_BASE_UUID_128(MOVEMENT_CONF_UUID),
};

/* Characteristic UUID: period */
static uint8_t sensorPeriodUUID[SNP_128BIT_UUID_SIZE] =
{
    TI_BASE_UUID_128(MOVEMENT_PERI_UUID),
};

static BLEProfileCallbacks_t *movementCBs;
static uint8_t cccdFlag = 0;
static uint16_t connHandle = 0;

/*******************************************************************************
 *                          Profile Attributes
 ******************************************************************************/
static SAP_Service_t movementService;
static SAP_CharHandle_t movementServiceCharHandles[MOVEMENT_NUM_ATTR_SUPPORTED];

/* Profile Service attribute */
static UUIDType_t sensorService =
{
     SNP_128BIT_UUID_SIZE,
     sensorServiceUUID
};

/* Characteristic Value: data */
static uint8_t sensorData[MOVEMENT_DATA_LEN] =
{
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/* Characteristic Value: configuration */
static uint8_t sensorCfg = 0;

/* Characteristic Value: period */
static uint16_t sensorPeriod = (MOVEMENT_MIN_UPDATE_PERIOD
        / MOVEMENT_PERIOD_RESOLUTION);

/* Characteristic User Description: data */
static uint8_t sensorDataUserDescr[] = MOVEMENT_DATA_DESCR;

/* Characteristic User Description: configuration */
static uint8_t sensorCfgUserDescr[] = MOVEMENT_CONFIG_DESCR;

/* Characteristic User Description: period */
static uint8_t sensorPeriodUserDescr[] = MOVEMENT_PERIOD_DESCR;

/*******************************************************************************
 *                              Profile Attributes - TABLE
 ******************************************************************************/
static SAP_UserDescAttr_t sensorDataUserDesc =
{
    SNP_GATT_PERMIT_READ,
    sizeof(sensorDataUserDescr),
    sizeof(sensorDataUserDescr),
    sensorDataUserDescr
};

static SAP_UserDescAttr_t sensorCfgUserDesc =
{
    SNP_GATT_PERMIT_READ,
    sizeof(sensorCfgUserDescr),
    sizeof(sensorCfgUserDescr),
    sensorCfgUserDescr
};

static SAP_UserDescAttr_t sensorPeriodUserDesc =
{
    SNP_GATT_PERMIT_READ,
    sizeof(sensorPeriodUserDescr),
    sizeof(sensorPeriodUserDescr),
    sensorPeriodUserDescr
};

static SAP_UserCCCDAttr_t sensorDataCCCD =
{
    SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE
};

static SAP_Char_t sensorAttrTable[MOVEMENT_NUM_ATTR_SUPPORTED] =
{
    /* Characteristic Value "Data" */
    {
        { SNP_128BIT_UUID_SIZE, sensorDataUUID }, /* UUID */
        SNP_GATT_PROP_NOTIFICATION, 0,            /* Properties */
        &sensorDataUserDesc,                      /* User Description */
        &sensorDataCCCD                           /* CCCD */
    },

    /* Characteristic Value "Configuration" */
    {
        { SNP_128BIT_UUID_SIZE, sensorCfgUUID },        /* UUID */
        SNP_GATT_PROP_READ | SNP_GATT_PROP_WRITE,       /* Properties */
        SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE,   /* Permissions */
        &sensorCfgUserDesc                              /* User Description */
    },
    /* Characteristic Value "Period" */
    {
        { SNP_128BIT_UUID_SIZE, sensorPeriodUUID },    /* UUID */
        SNP_GATT_PROP_READ | SNP_GATT_PROP_WRITE,      /* Properties */
        SNP_GATT_PERMIT_READ | SNP_GATT_PERMIT_WRITE,  /* Permissions */
        &sensorPeriodUserDesc                          /* User Description */
    },
};

/*******************************************************************************
 *                                  LOCAL FUNCTIONS
 ******************************************************************************/
static void sensor_processSNPEventCB(uint16_t event, snpEventParam_t *param);
static uint8_t sensor_ReadAttrCB(void *context, uint16_t conn, uint16_t charHdl,
                                 uint16_t offset, uint16_t size,
                                 uint16_t * pLen, uint8_t *pData);
static uint8_t sensor_WriteAttrCB(void *context, uint16_t conn,
                                  uint16_t charHdl, uint16_t len,
                                  uint8_t *pData);
static uint8_t sensor_CCCDIndCB(void *context, uint16_t conn, uint16_t cccdHdl,
                                uint8_t type, uint16_t value);

/*******************************************************************************
 *                                 PUBLIC FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn      Movement_addService
 *
 * @brief   Initializes the Movement Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 ******************************************************************************/
uint8_t Movement_addService(void)
{
    /* Register to receive connection established events */
    SAP_registerEventCB(sensor_processSNPEventCB,
            SNP_CONN_EST_EVT | SNP_CONN_TERM_EVT);

    /* Build Service to register with NP */
    movementService.serviceUUID = sensorService;
    movementService.serviceType = SNP_PRIMARY_SERVICE;
    movementService.charTableLen = MOVEMENT_NUM_ATTR_SUPPORTED;
    movementService.charTable = sensorAttrTable;
    movementService.context = NULL;
    movementService.charReadCallback = sensor_ReadAttrCB;
    movementService.charWriteCallback = sensor_WriteAttrCB;
    movementService.cccdIndCallback = sensor_CCCDIndCB;
    movementService.charAttrHandles = movementServiceCharHandles;

    /* Service is set up, register with GATT server on the SNP. */
    return SAP_registerService(&movementService);
}

/*******************************************************************************
 * @fn      Movement_registerAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  BLE_PROFILE_SUCCESS or BLE_PROFILE_ALREADY_IN_REQ_MODE
 ******************************************************************************/
uint8_t Movement_registerAppCBs(BLEProfileCallbacks_t *appCallbacks)
{
    if (movementCBs == NULL)
    {
        if (appCallbacks != NULL)
        {
            movementCBs = appCallbacks;
        }

        return BLE_PROFILE_SUCCESS;
    }

    return BLE_PROFILE_ALREADY_IN_REQ_MODE;
}

/*******************************************************************************
 * @fn      Movement_setParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  uint8_t
 ******************************************************************************/
uint8_t Movement_setParameter(uint8_t param, uint8_t len, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;

    switch (param)
    {
    case MOVEMENT_DATA:
        if (len == MOVEMENT_DATA_LEN)
        {
            snpNotifIndReq_t localReq;
            memcpy(sensorData, value, MOVEMENT_DATA_LEN);

            /* Initialize Request */
            localReq.connHandle = connHandle;
            localReq.attrHandle = ProfileUtil_getHdlFromCharID(MOVEMENT_DATA_ID,
                    movementServiceCharHandles,
                    MOVEMENT_NUM_ATTR_SUPPORTED);
            localReq.pData = (uint8_t *) &sensorData[0];
            localReq.authenticate = 0; // Default no authentication

            /* Check for whether a notification or indication should be sent.
               Both flags should never be allowed to be set by NWP. */
            if (cccdFlag & SNP_GATT_CLIENT_CFG_NOTIFY)
            {
                localReq.type = SNP_SEND_NOTIFICATION;
                SNP_RPC_sendNotifInd(&localReq, sizeof(sensorData));
            }
            else if (cccdFlag & SNP_GATT_CLIENT_CFG_INDICATE)
            {
                localReq.type = SNP_SEND_INDICATION;
                SNP_RPC_sendNotifInd(&localReq, sizeof(sensorData));
            }
        }
        else
        {
            ret = BLE_PROFILE_INVALID_RANGE;
        }
        break;

    case MOVEMENT_CONF:
        if (len == MOVEMENT_DATA_LEN)
        {
            sensorCfg = *((uint8_t*) value);
        }
        else
        {
            ret = BLE_PROFILE_INVALID_RANGE;
        }
        break;

    case MOVEMENT_PERI:
        if (len == sizeof(uint16_t))
        {
            sensorPeriod = *((uint16_t*) value);
        } else
        {
            ret = BLE_PROFILE_INVALID_RANGE;
        }
        break;

    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }

    return (ret);
}

/*******************************************************************************
 * @fn      Movement_getParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16_t will be cast to
 *          uint16_t pointer).
 *
 * @return  uint8_t
 ******************************************************************************/
uint8_t Movement_getParameter(uint8_t param, void *value)
{
    uint8_t ret = BLE_PROFILE_SUCCESS;

    switch (param)
    {
    case MOVEMENT_DATA:
        memcpy(value, sensorData, MOVEMENT_DATA_LEN);
        break;

    case MOVEMENT_CONF:
        *((uint8_t*) value) = sensorCfg;
        break;

    case MOVEMENT_PERI:
        *((uint16_t*) value) = sensorPeriod;
        break;

    default:
        ret = BLE_PROFILE_INVALIDPARAMETER;
        break;
    }

    return (ret);
}

/*******************************************************************************
 * @fn      sensor_ReadAttrCB
 *
 * @brief   Read an attribute.
 *
 * @param   context - context used when registering service
 * @param   conn    - connection handle ID
 * @param   charHdl - characteristic value handle
 * @param   offset  - offset of data to be read
 * @param   size    - maximum size of data bytes to be read
 * @param   pLen    - amount of bytes copied into pData
 * @param   pData   - pointer to copy read data
 *
 * @return  SNP_UNKNOWN_ATTRIBUTE
 ******************************************************************************/
static uint8_t sensor_ReadAttrCB(void *context, uint16_t connectionHandle,
        uint16_t charHdl, uint16_t offset, uint16_t size, uint16_t * len,
        uint8_t *pData)
{
    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
            movementServiceCharHandles,
            MOVEMENT_NUM_ATTR_SUPPORTED);
    uint8_t isValid = 0;

    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    switch (PROFILE_ID_CHAR(charID))
    {
    case MOVEMENT_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *len = MOVEMENT_DATA_LEN;
            memcpy(pData, sensorData, MOVEMENT_DATA_LEN);
            isValid = 1;
            break;

        default:
            break;
        }
        break;

    case MOVEMENT_CONF:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *len = sizeof(sensorCfg);
            memcpy(pData, &sensorCfg, sizeof(sensorCfg));
            isValid = 1;
            break;

        default:
            break;
        }
        break;

    case MOVEMENT_PERI:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            *len = sizeof(sensorPeriod);
            memcpy(pData, &sensorPeriod, sizeof(sensorPeriod));
            isValid = 1;
            break;

        default:
            break;
        }
        break;
    default:
        break;
    }

    if (isValid)
    {
        return BLE_PROFILE_SUCCESS;
    }

    /* Unable to find handle - set len to 0 and return error code */
    *len = 0;
    return SNP_UNKNOWN_ATTRIBUTE;
}

/*******************************************************************************
 * @fn      sensor_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  BLE_PROFILE_SUCCESS, blePending or Failure
 ******************************************************************************/
static uint8_t sensor_WriteAttrCB(void *context, uint16_t connectionHandle,
        uint16_t charHdl, uint16_t len, uint8_t *pData)
{
    uint8_t status = SNP_UNKNOWN_ATTRIBUTE;
    uint8_t notifyApp = PROFILE_UNKNOWN_CHAR;

    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(charHdl,
            movementServiceCharHandles,
            MOVEMENT_NUM_ATTR_SUPPORTED);

    switch (PROFILE_ID_CHAR(charID))
    {
    case MOVEMENT_DATA:
        break;

    case MOVEMENT_CONF:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            if (len == sizeof(sensorCfg))
            {
                sensorCfg = pData[0];
                status = BLE_PROFILE_SUCCESS;
                notifyApp = MOVEMENT_CONF;
            }
            break;
        default:
            break;
        }
        break;

    case MOVEMENT_PERI:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_VALUE:
            sensorPeriod = (pData[1] << 8) | pData[0];
            status = BLE_PROFILE_SUCCESS;
            notifyApp = MOVEMENT_PERI;
            break;
        default:
            break;
        }
        break;

    default:
        break;
    }

    /* If a characteristic value changed then callback function to notify
     * application of change.
     */
    if ((notifyApp != PROFILE_UNKNOWN_CHAR) && movementCBs
            && movementCBs->charChangeCB)
    {
        movementCBs->charChangeCB(notifyApp);
    }

    return status;
}

/*******************************************************************************
 * @fn      sensor_CCCDIndCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  BLE_PROFILE_SUCCESS, blePending or Failure
 ******************************************************************************/
uint8_t sensor_CCCDIndCB(void *context, uint16_t connectionHandle,
        uint16_t cccdHdl, uint8_t type, uint16_t value)
{
    uint8_t status = SNP_UNKNOWN_ATTRIBUTE;
    uint8_t notifyApp = PROFILE_UNKNOWN_CHAR;

    /* Update connection handle (assumes one connection) */
    connHandle = connectionHandle;

    /* Get characteristic from handle */
    uint8_t charID = ProfileUtil_getCharIDFromHdl(cccdHdl,
            movementServiceCharHandles,
            MOVEMENT_NUM_ATTR_SUPPORTED);

    switch (PROFILE_ID_CHAR(charID))
    {
    case MOVEMENT_DATA:
        switch (PROFILE_ID_CHARTYPE(charID))
        {
        case PROFILE_CCCD:
            /* Set Global cccd Flag which will be used to to gate Indications
               or Notifications when SetParameter() is called */
            cccdFlag = value;
            notifyApp = charID;
            status = BLE_PROFILE_SUCCESS;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    /* If a characteristic value changed then callback function to notify
     * application of change.
     */
    if ((notifyApp != PROFILE_UNKNOWN_CHAR) && movementCBs
            && movementCBs->cccdUpdateCB)
    {
        movementCBs->cccdUpdateCB(notifyApp, value);
    }

    return status;
}

/*******************************************************************************
 * @fn      sensor_processSNPEventCB
 *
 * @brief   This is a callback operating in the NPI task. It will be invoked
 *          whenever an event is received from the SNP that this profile has
 *          registered for
 *
 * @param   event  - event mask
 * @param   pValue - pointer event struct
 *
 * @return  status
 ******************************************************************************/
static void sensor_processSNPEventCB(uint16_t event, snpEventParam_t *param)
{
    switch (event)
    {
    case SNP_CONN_EST_EVT:
    {
    }
        break;

    case SNP_CONN_TERM_EVT:
    {
    }
        break;

    default:
        break;
    }
}

