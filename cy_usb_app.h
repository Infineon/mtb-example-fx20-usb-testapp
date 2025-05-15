/***************************************************************************//**
* \file cy_usb_app.h
* \version 1.0
*
* Defines the interfaces used in the FX20 USB test application.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _CY_USB_APP_H_
#define _CY_USB_APP_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h"
#include "cy_debug.h"
#include "cy_hbdma_mgr.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define TESTDEV_CFG_STRUCT_ADDRESS      (0x08001F00UL)  /* RAM location where test configuration should be stored. */
#define TESTDEV_CFG_START_SIG           (0x53474643UL)  /* Configuration start signature: "CFGS" */
#define TESTDEV_CFG_END_SIG             (0x45474643UL)  /* Configuration end signature: "CFGE" */

typedef struct {
    uint32_t startSig;                  /* Start signature for the config structure. */

    uint8_t  connSpeed;                 /* Desired connection speed: 0 = Gen1, 1 = Gen2, 2 = HS, 3 = FS */
    bool     useWinUsb;                 /* Whether WinUSB driver is to be selected. */
    uint8_t  numEps;                    /* Number of endpoints supported in each direction. */
    uint8_t  clkConfig;                 /* DUT clock configuration:
                                           b0: When set to 1, sets HFCLK frequency to 50 MHz (for PSVP use only).
                                           b[2:1]: DMA clock setting (for Silicon use only):
                                               0 ==> 240 MHz derived from USBHS PLL
                                               1 ==> 160 MHz derived from USBHS PLL
                                               2 ==> 120 MHz derived from USBHS PLL
                                               3 ==> 150 MHz derived from HFCLK
                                         */

    uint8_t  vBusDetEnabled;            /* Power settings:
                                           b0: Whether VBus detection is to be enabled.
                                           b1: Whether deep sleep in L2/U3 state is enabled.
                                         */
    uint8_t  ebDepth;                   /* Elastic buffer half depth setting to be used in case of Gen2 connection. */
    uint16_t isoEpMask;                 /* Bit mask specifying EPs that should be configured as Isochronous. */

    uint16_t bulkBufSize;               /* DMA buffer size to be used for Bulk Endpoints. */
    uint16_t bulkPktSize;               /* Max packet size to be used for Bulk Endpoints. */ 

    uint8_t  bulkMaxBurst;              /* Max burst setting to be used for Bulk Endpoints. */
    uint8_t  bulkBufCount;              /* Number of buffers to be used for Bulk Endpoints. */
    uint8_t  dmaConfig;                 /* DMA configuration settings:
                                           b0: Enable burst-mode
                                           b1: Use AUTO DMA for loopback
                                         */
    uint8_t  ccPollEnable;              /* Whether CC based connect detect is enabled. */

    uint16_t isoBufSize;                /* Buffer size to be used for Isochronous endpoints. */
    uint16_t isoPktSize;                /* Max packet size for Isochronous endpoints. */

    uint8_t  isoMaxBurst;               /* Max burst setting to be used for Isochronous Endpoints. */
    uint8_t  isoBurstCnt;               /* Number of bursts (1 - 3) per interval for Isochronous Endpoints. */
    uint8_t  isoPollRate;               /* Polling rate for Isochronous Endpoints. */
    uint8_t  isoBufCount;               /* Number of buffers to be used for Isochronous Endpoints. */

    uint32_t endSig;                    /* End signature for the config structure. */
} cy_stc_testdev_config_t;

/*
 * List of vendor commands used with this firmware application.
 */
typedef enum {
    REG_MEMORY_READ_CODE   = 0xA0,      /* Request used to read registers and/or memory. */
    MEMORY_WRITE_CODE      = 0xA1,      /* Request used to write to SRAM locations. */
    DATA_XFER_TEST_CODE    = 0xB8,      /* Request used to test EP0 data transfer. */
    USB_LPM_TEST_CODE      = 0xB9,      /* Request used to test USB3 LPM transitions. */
    DATA_BUF_READ_CODE     = 0xBA,      /* Request used to read the contents of the EP loop-back data buffer. */
    VBUS_REMOVE_SIM_CODE   = 0xBB,      /* Request used to initiate simulation of VBus removal (for test). */
    DEVICE_RESET_CODE      = 0xE0,      /* Request to reset the device. */
    DEVICE_RE_ENUM_CODE    = 0xE1,      /* Request the device to disconnect and reconnect after a delay. */
    IN_EP_CRCERR_INJ_CODE  = 0xE2,      /* Request to trigger CRC-32 error injection on USB IN EP data transfers. */
    UPDT_TEST_CONFIG_CODE  = 0xE3,      /* Request to update the default configuration and reset the device. */
    MS_VENDOR_CODE         = 0xF0,      /* Request used to fetch MS-OS descriptors. */
    GET_EPCONF_CMD_CODE    = 0xF4,      /* Fetch EP configuration information. */
    GET_DEVSPEED_CMD       = 0xF6,      /* Command to get device speed information from the device. */
    BOOT_MODE_RQT_CODE     = 0xF7,      /* Command to force device to return to bootloader mode. */
} cy_usb_app_vendor_opcode_t;

extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

#define MAX_LP_PAIRS_DMAC       (2u)            /* Maximum number of Loop-back pairs supported when DMAC is used. */

#define MAX_EP_BUF_CNT          (64u)           /* Maximum number of buffers used per endpoint/pair. */
#define DFLT_BUF_CNT            (1u)            /* Default number of buffers used per endpoint/pair. */
#define DFLT_BUF_SIZE           (4096u)         /* Default size of buffers used for data loopback. */

#define EV_DEVSTATE_CHG         (0x0001U)       /* Event bit which indicates device state change. */
#define EV_LOOPBACK_PAIR1       (0x0002U)       /* Event bit which indicates loopback pair #1 has pending work. */
#define EV_LOOPBACK_PAIR2       (0x0004U)       /* Event bit which indicates loopback pair #2 has pending work. */
#define EV_LOOPBACK_PAIR3       (0x0008U)       /* Event bit which indicates loopback pair #3 has pending work. */
#define EV_LOOPBACK_PAIR4       (0x0010U)       /* Event bit which indicates loopback pair #4 has pending work. */
#define EV_LOOPBACK_PAIR5       (0x0020U)       /* Event bit which indicates loopback pair #5 has pending work. */
#define EV_LOOPBACK_PAIR6       (0x0040U)       /* Event bit which indicates loopback pair #6 has pending work. */
#define EV_LOOPBACK_PAIR7       (0x0080U)       /* Event bit which indicates loopback pair #7 has pending work. */
#define EV_LOOPBACK_PAIR8       (0x0100U)       /* Event bit which indicates loopback pair #8 has pending work. */
#define EV_LOOPBACK_PAIR9       (0x0200U)       /* Event bit which indicates loopback pair #9 has pending work. */
#define EV_LOOPBACK_PAIR10      (0x0400U)       /* Event bit which indicates loopback pair #10 has pending work. */
#define EV_LOOPBACK_PAIR11      (0x0800U)       /* Event bit which indicates loopback pair #11 has pending work. */
#define EV_LOOPBACK_PAIR12      (0x1000U)       /* Event bit which indicates loopback pair #12 has pending work. */
#define EV_LOOPBACK_PAIR13      (0x2000U)       /* Event bit which indicates loopback pair #13 has pending work. */
#define EV_LOOPBACK_PAIR14      (0x4000U)       /* Event bit which indicates loopback pair #14 has pending work. */
#define EV_LOOPBACK_PAIR15      (0x8000U)       /* Event bit which indicates loopback pair #15 has pending work. */
#define EV_VENDOR_REQUEST       (0x10000U)      /* Event bit which indicates that EP0 vendor request is pending. */
#define TASK_WAIT_EV_MASK       (EV_DEVSTATE_CHG | EV_VENDOR_REQUEST | \
        EV_LOOPBACK_PAIR1 | EV_LOOPBACK_PAIR2 | EV_LOOPBACK_PAIR3 | EV_LOOPBACK_PAIR4 | \
        EV_LOOPBACK_PAIR5 | EV_LOOPBACK_PAIR6 | EV_LOOPBACK_PAIR7 | EV_LOOPBACK_PAIR8 | \
        EV_LOOPBACK_PAIR9 | EV_LOOPBACK_PAIR10 | EV_LOOPBACK_PAIR11 | EV_LOOPBACK_PAIR12 | \
        EV_LOOPBACK_PAIR13 | EV_LOOPBACK_PAIR14 | EV_LOOPBACK_PAIR15 \
        )

/* Maximum size of configuration descriptor. */
#define MAX_HS_CFG_DSCR_SIZE       (9u + 9u + 15u * 2u * 7u)
#define MAX_SS_CFG_DSCR_SIZE       (9u + 9u + 15u * 2u * (7u + 6u))

/* Map blocking delay call to appropriate function based on RTOS enable/disable */
#if FREERTOS_ENABLE
#define AppBlockingDelay        vTaskDelay
#else
#define AppBlockingDelay        Cy_SysLib_Delay
#endif /* FREERTOS_ENABLE */

#define USB3_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))
#define HBDMA_BUF_ATTRIBUTES __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32)))

/* Data structure used to specify all config parameters for a pair of endpoints. */
typedef struct cy_stc_app_ep_conf
{
    uint8_t               outEp;                /* Out EP index. */
    uint8_t               inEp;                 /* In EP index. */
    bool                  isLpbk;               /* Whether EPs are configured as loop-back endpoints. */
    cy_en_usb_endp_type_t epType;               /* Type of endpoint. */
    uint16_t              mPktSizeSS;           /* Max. Pkt. Size for USB SuperSpeed. */
    uint16_t              mPktSizeHS;           /* Max. Pkt. Size for USB High Speed. */
    uint16_t              mPktSizeFS;           /* Max. Pkt. Size for USB Full Speed. */
    uint8_t               pollRate;             /* Polling rate to be set for INTR/ISO endpoints. */
    uint8_t               maxBurst;             /* Max. Burst Size to be used in SS EP Companion descriptor. */
    uint8_t               ssAttrib;             /* bmAttributes to be used in SS EP Companion descriptor.
                                                   In case of Bulk EP, specifies number of streams.
                                                   In case of INTR/ISO, specified MULT field.
                                                   wBytesPerInterval will be calculated from the values specified. */
    uint8_t               numBuff;              /* Number of SRAM buffers to be used. */
    uint16_t              buffSize;             /* Size of each SRAM buffer. */
} cy_stc_app_ep_conf_t;

/*
 * Data structure that encapsulates all information about a pair of bulk loop-back endpoints.
 */
typedef struct LoopBackContext_
{
    bool     LpbkEnable;                /* Whether EP pair is configured for loopback. */
    uint8_t  OutEndpNum;                /* OUT endpoint index. */
    uint8_t  InEndpNum;                 /* IN endpoint index. */
    bool     BulkOutDmaDone;            /* Whether OUT DMA transfer has been completed. */
    bool     BulkInDmaDone;             /* Whether IN DMA transfer has been completed. */
    bool     rdQueued;                  /* Whether OUT EP read operation is pending. */
    bool     wrQueued;                  /* Whether IN EP write operation is pending. */
    bool     zlpRcvd;                   /* Whether the last OUT packet was a ZLP. */
    bool     slpRcvd;                   /* Whether the last OUT packet was an SLP. */
    uint8_t  bufferCnt;                 /* Number of buffers allocated for this EP pair. */
    uint8_t  packetCnt;                 /* Number of RAM buffers that are currently occupied. */
    uint8_t  nextRdIdx;                 /* Index of next OUT EP read buffer. */
    uint8_t  nextWrIdx;                 /* Index of next IN EP write buffer. */
    uint32_t bufferSize;                /* Size of each DMA buffer for this EP pair. */
    uint16_t packetLen[MAX_EP_BUF_CNT]; /* Amount of data in each occupied RAM buffer. */
    uint16_t slpLen;                    /* If short length packet has been received, provides its size in bytes. */

    uint8_t *outEpBuf[MAX_EP_BUF_CNT];  /* Pointer to RAM buffers used for OUT EP read. */
    uint8_t *inEpBuf[MAX_EP_BUF_CNT];   /* Pointer to RAM buffers used for IN EP write. In loopback case, will be
                                           the same as outEpBuf pointers. */
} LoopBackContext_t;

/* 
 * USB application data structure which is bridge between USB system and device
 * functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    cy_en_usb_device_state_t devState;
    cy_en_usb_device_state_t prevDevState;
    cy_en_usb_speed_t devSpeed;
    uint8_t devAddr;
    uint8_t activeCfgNum;
    cy_en_usb_enum_method_t enumMethod;
    uint8_t prevAltSetting;
    cy_en_usb_speed_t desiredSpeed;

    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];

    DMAC_Type *pCpuDmacBase;
    DW_Type *pCpuDw0Base;
    DW_Type *pCpuDw1Base;
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;

    bool    usbConnectEnabled;
    bool    usbConnectDone;
    bool    vbusChangeIntr;
    bool    vbusPresent;
    bool    winusbEnabled;
    bool    burstModeEnable;
    bool    autoDmaEnable;
    bool    interruptEpEnable;
    bool    functionWakeEnable;
    uint8_t maxLbkPairs;
    uint8_t numLbkPairs;
    uint32_t reconnectTimeStamp;
    uint32_t functionWakeTimestamp;
    LoopBackContext_t loopbackInfo[15u];
    TaskHandle_t appTaskHandle;
    TaskHandle_t logTaskHandle;
    EventGroupHandle_t appEvGrpHandle;

    cy_stc_hbdma_mgr_context_t *pHbDmaMgr;              /* Pointer to HBW DMA manager context structure. */
    uint32_t *pUsbEvtLogBuf;

    TimerHandle_t ccPollTimer;                          /* Timer used to poll CC pin voltage. */
    bool          ccDisconnectDetect;                   /* Whether CC disconnect is detected. */
    bool          isLpmEnabled;                         /* Whether LPM transitions are enabled. */
    uint32_t      lpmEnableTime;                        /* Timestamp at which LPM should be re-enabled. */
    bool          isAppActive;                          /* Whether application is active from data perspective. */
};

void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, 
        DMAC_Type *pCpuDmacBase,
        DW_Type *pCpuDw0Base,
        DW_Type *pCpuDw1Base,
        cy_stc_hbdma_mgr_context_t *pHbDmaMgr);

void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);
uint32_t *Cy_USB_CalculateEpmAddr(uint32_t endpNum, cy_en_usb_endp_dir_t endpDirection);

void Cy_USB_AppSetCfgCallback(void *pAppCtxt,
                              cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt, 
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, 
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetupCallback(void *pAppCtxt, 
                             cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppEp0RecvCallback(void *pAppCtxt,
                                 cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                 cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSuspendCallback(void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppResumeCallback (void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, 
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1SleepCallback(void *pUsbApp,
                               cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppL1ResumeCallback(void *pUsbApp,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppZlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);
void Cy_USB_AppSlpCallback(void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg);

void Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);
uint16_t Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize);
void Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize);
void Cy_USB_AppInitDmaIntr(cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t endpNumber,
        cy_en_usb_endp_dir_t endpDirection, cy_israddress userIsr);
void Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection);
void Cy_USB_AppTerminateDma(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, cy_en_usb_endp_dir_t endpDirection);

uint8_t Cy_USB_AppFindValidInEndpNumber(cy_stc_usb_app_ctxt_t *pUsbApp);
uint8_t Cy_USB_AppFindValidOutEndpNumber(cy_stc_usb_app_ctxt_t *pUsbApp);
uint32_t Cy_USB_AppGetMaxPktSize(cy_stc_usb_app_ctxt_t *pUsbApp,
                                 uint8_t endpNum, cy_en_usb_endp_dir_t dir);

bool Cy_USB_AppSignalTask(cy_stc_usb_app_ctxt_t *pAppCtxt, const EventBits_t evMask);

/*  Eco device related defines */
#define CY_USB_DEV_ECO_BREQ_RESET                  (0xE0)
#define CY_USB_DEV_ECO_BREQ_RE_ENUM                (0xE1)
#define CY_USB_DEV_ECO_BREQ_CAPABILITY             (0xF0)

#define CY_USB_DEV_ECO_BMREQ_RESET                  (0x40)
#define CY_USB_DEV_ECO_BMREQ_RE_ENUM                (0x40)
#define CY_USB_DEV_ECO_BMREQ_CAPABILITY             (0xC0)

#define CY_USB_DEV_ECO_SPEED_FS                    (0x01)
#define CY_USB_DEV_ECO_SPEED_HS                    (0x02)
#define CY_USB_DEV_ECO_SPEED_GEN1X1                (0x08)
#define CY_USB_DEV_ECO_SPEED_GEN1X2                (0x0C)
#define CY_USB_DEV_ECO_SPEED_GEN2X1                (0x10)
#define CY_USB_DEV_ECO_SPEED_GEN2X2                (0x14)

/* USBD layer return code shared between USBD layer and Application layer. */
typedef enum cy_en_usb_app_ret_code_ {
    CY_USB_APP_STATUS_SUCCESS=0,
    CY_USB_APP_STATUS_FAILURE,
}cy_en_usb_app_ret_code_t;

/* Function to do free up DMA resources as part of disconnection. */
void Cy_USB_AppDisableEndpDma(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Function to print a string to UART console. */
void PrintString(const char *string, uint8_t length);

/* Function to print a 32-bit unsigned integer value to the UART console in HEX format. */
void PrintDword(uint32_t data);

/* Function to print an 8-bit unsigned integer value to the UART console in HEX format. */
void PrintByte(uint8_t data);

/* Function to print the contents of a 8-bit unsigned data buffer to UART console. */
void PrintBuffer(uint8_t *buf_p, uint16_t len);

/* Function to print a register name and value (in HEX format) to the UART console. */
void PrintReg(const char *name, uint8_t namelen, uint32_t value);

/* Function to print the contents of the USB event log buffer. */
void AppPrintUsbEventLog (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usbss_cal_ctxt_t *pSSCal);

/* Function to register all USB descriptors with the stack. */
void CyApp_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_usb_speed_t usbSpeed);

/* Function to check whether configuration descriptor length is within expected range. */
bool CyApp_CheckConfigDescriptorLength(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Function to add a pair of endpoints to all configuration descriptors. */
void CyApp_AddEpPairToCfgDscr(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_app_ep_conf_t *pEpConf);

/* Function to revert all configuration descriptors to their original value. */
void CyApp_RevertConfigDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt);

/* Function to update the product ID to be used in the device descriptors. */
void CyApp_SetProductId(cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t pidVal);

/* Non-RTOS function to mark pending work on endpoint pair. */
void Cy_App_MarkEpPairPending(uint8_t lpPairIdx);

/* Non-RTOS function to clear pending work on endpoint pair. */
void Cy_App_ClearEpPairPending(uint8_t lpPairIdx);

/* Function to handle vendor specific requests. */
void Cy_USB_AppVendorRqtHandler(cy_stc_usb_app_ctxt_t *pAppCtxt);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USB_APP_H_ */

/* End of File */

