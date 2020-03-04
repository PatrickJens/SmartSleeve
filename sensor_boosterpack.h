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
#ifndef SimpleBP_H
#define SimpleBP_H

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 *                                FUNCTIONS
 ******************************************************************************/
extern void AP_createTask(void);
extern const unsigned short SNP_code[];

/*******************************************************************************
 *                            CONSTANTS
 ******************************************************************************/
/* Advertising interval when device is discoverable (625us, 160=100ms) */
#define DEFAULT_ADVERTISING_INTERVAL          160

/* Limited discoverable mode advertises for 30.72s, and then stops
   general discoverable mode advertises indefinitely */
#define DEFAULT_DISCOVERABLE_MODE             SAP_GAP_ADTYPE_FLAGS_GENERAL

/* Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
   parameter update request is enabled */
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     80

/* Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
   parameter update request is enabled */
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800

/**
 * AP States.
 */
typedef enum
{
    AP_RESET,
    AP_IDLE,
    AP_START_ADV,
    AP_CONNECTED,
    AP_CANCEL_ADV,
    AP_OAD,
    AP_SBL
} ap_States_t;

/* Task configuration */
#define AP_TASK_PRIORITY                     1
#define AP_TASK_STACK_SIZE                   2048

/* Application Events */
#define AP_NONE                              0x00000000
#define AP_EVT_PUI                           0x00000001
#define AP_EVT_ADV_ENB                       0x00000002
#define AP_EVT_ADV_END                       0x00000004
#define AP_EVT_CONN_EST                      0x00000008
#define AP_EVT_CONN_TERM                     0x00000010
#define AP_EVT_HCI_BDADDR                    0x00000020
#define AP_EVT_BSL_BUTTON                    0x00000040
#define AP_EVT_BUTTON_RESET                  0x00000080
#define AP_EVT_BUTTON_RIGHT                  0x00000100
#define AP_ERROR                             0x00000200

/* How often to perform periodic event (in msec) */
#define AP_PERIODIC_EVT_PERIOD               5000
#define AP_DEFAULT_CONN_HANDLE               0xFFFF

/* Company Identifier: Texas Instruments Inc. (13) */
#define TI_COMPANY_ID                        0x000D
#define TI_ST_DEVICE_ID                      0x03
#define TI_ST_KEY_DATA_ID                    0x00


#ifdef __cplusplus
}
#endif

#endif /* SimpleBP_H */
