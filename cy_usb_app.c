/***************************************************************************//**
* \file cy_usb_app.c
* \version 1.0
*
* Implements the USB data handling part of the FX20 USB test application.
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "event_groups.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usbss_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usbhs_dmac_wrapper.h"
#include "cy_fx_common.h"
#include <string.h>

/* Buffer used for control transfer testing. */
HBDMA_BUF_ATTRIBUTES uint32_t Ep0TestBuffer[2048U];

/* Variables used for SET_SEL control request handling. */
HBDMA_BUF_ATTRIBUTES uint32_t SetSelDataBuffer[8U];
volatile bool SetSelRqtPending = false;


/*
 * Function     : Cy_USB_IsValidMMIOAddr()
 * Description :  Check if the passed address is within the valid MMIO or System RAM address range.
 *                Note that this is not an exhaustive check as the MMIO range is not contiguous.
 * Parameters  :  const uint32_t address
 * Return      :  bool
 */
static bool Cy_USB_IsValidMMIOAddr (const uint32_t address)
{
    uint32_t periLastAddr = 0x700000;

    if (
            ((address >= PERI_BASE) && (address < (PERI_BASE + periLastAddr))) ||
            ((address >= CY_SRAM_BASE) && (address < (CY_SRAM_BASE + CY_SRAM_SIZE)))
       )
    {
        return true;
    }
    else
    {
        return false;
    }
}

/*
 * Function: Cy_USB_AppInit()
 * Description: This function Initializes application related data structure.
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void
Cy_USB_AppInit (cy_stc_usb_app_ctxt_t *pAppCtxt,
        cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, DMAC_Type *pCpuDmacBase,
        DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
        cy_stc_hbdma_mgr_context_t *pHbDmaMgr)
{
    uint32_t index;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->enumMethod = CY_USB_ENUM_METHOD_FAST;
    pAppCtxt->pHbDmaMgr = pHbDmaMgr;
    pAppCtxt->isAppActive = false;

    for (index=0x00; index < CY_USB_MAX_ENDP_NUMBER; index++) {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
    }

    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;

    /* Zero out the EP0 test buffer. */
    memset ((uint8_t *)Ep0TestBuffer, 0, sizeof(Ep0TestBuffer));

    return;
}   /* end of function. */

/*
 * Function: CyApp_GetLoopbackPairIdx()
 * Description: Function to look up the loopback pair index corresponding
 * to a given OUT or IN endpoint.
 * Parameter: cy_stc_usb_app_ctxt_t *pUsbApp, cy_en_usb_endp_dir_t endpDir, uint8_t endpNum
 * Return: Index of the loop back endpoint pair.
 */
uint8_t CyApp_GetLoopbackPairIdx (cy_stc_usb_app_ctxt_t *pUsbApp,
        cy_en_usb_endp_dir_t endpDir, uint8_t endpNum)
{
    uint8_t lpPairIdx;

    /* Look up the ep pair index corresponding to the endpoint. */
    for (lpPairIdx = 0; lpPairIdx < pUsbApp->numLbkPairs; lpPairIdx++) {
        if (endpDir == CY_USB_ENDP_DIR_OUT) {
            if (pUsbApp->loopbackInfo[lpPairIdx].OutEndpNum == endpNum) {
                break;
            }
        } else {
            if (pUsbApp->loopbackInfo[lpPairIdx].InEndpNum == endpNum) {
                break;
            }
        }
    }

    return (lpPairIdx);
}
/*
 * Function: CyApp_GetLoopbackInfo()
 * Description: Function to look up the loopback context structure corresponding
 * to a given OUT or IN endpoint.
 * Parameter: cy_stc_usb_app_ctxt_t *pUsbApp, cy_en_usb_endp_dir_t endpDir, uint8_t endpNum
 * Return: Pointer to loopback context structure.
 */
static LoopBackContext_t *CyApp_GetLoopbackInfo (cy_stc_usb_app_ctxt_t *pUsbApp,
        cy_en_usb_endp_dir_t endpDir, uint8_t endpNum)
{
    LoopBackContext_t         *pLpbkCtxt = NULL;
    uint8_t                    lpPairIdx;

    /* Look up the ep pair index corresponding to the endpoint. */
    for (lpPairIdx = 0; lpPairIdx < pUsbApp->numLbkPairs; lpPairIdx++) {
        if (endpDir == CY_USB_ENDP_DIR_OUT) {
            if (pUsbApp->loopbackInfo[lpPairIdx].OutEndpNum == endpNum) {
                pLpbkCtxt = &(pUsbApp->loopbackInfo[lpPairIdx]);
                break;
            }
        } else {
            if (pUsbApp->loopbackInfo[lpPairIdx].InEndpNum == endpNum) {
                pLpbkCtxt = &(pUsbApp->loopbackInfo[lpPairIdx]);
                break;
            }
        }
    }

    return (pLpbkCtxt);
}

extern void
LoopBackApp_SLPCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg);
extern void
LoopBackApp_ZLPCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg);

/*
 * Function: Cy_USB_AppSSDisconnectCallback()
 * Description: This Function will be called by USBD layer when
 *              a SuperSpeed disconnect event is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSSDisconnectCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    /* No action to be taken here. */
    DBG_APP_TRACE("SSDisconnect callback\r\n");
}

/*
 * Function: Cy_USB_AppSetAddressCallback()
 * Description: This Function will be called by USBD layer when
 *              a USB address has been assigned to the device.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppSetAddressCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;

    /* Update the state variables. */
    pAppCtxt->devState     = CY_USB_DEVICE_STATE_ADDRESS;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DEFAULT;
    pAppCtxt->devAddr      = pUsbdCtxt->devAddr;
    pAppCtxt->devSpeed     = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    DBG_APP_INFO("Device address is %d\r\n", pAppCtxt->devAddr);

    /* Check the type of USB connection and register appropriate descriptors. */
    CyApp_RegisterUsbDescriptors(pAppCtxt, pAppCtxt->devSpeed);
}

/*
 * Function: Cy_USB_AppRegisterCallback()
 * Description: This function will register all calback with USBD layer.
 * Parameter: cy_stc_usb_app_ctxt_t.
 * return: None.
 */
void
Cy_USB_AppRegisterCallback (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET,
            Cy_USB_AppBusResetCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE,
            Cy_USB_AppBusResetDoneCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED,
            Cy_USB_AppBusSpeedCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP,
            Cy_USB_AppSetupCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_EP0_RCV_DONE,
            Cy_USB_AppEp0RecvCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND,
            Cy_USB_AppSuspendCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME,
            Cy_USB_AppResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG,
            Cy_USB_AppSetCfgCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF,
            Cy_USB_AppSetIntfCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_SLEEP,
            Cy_USB_AppL1SleepCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_L1_RESUME,
            Cy_USB_AppL1ResumeCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_ZLP,
            LoopBackApp_ZLPCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SLP,
            LoopBackApp_SLPCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_DISCONNECT,
            Cy_USB_AppSSDisconnectCallback);
    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETADDR,
            Cy_USB_AppSetAddressCallback);

    return;
}   /* end of function. */

static void Cy_USB_App_KeepLinkActive (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
#if USB3_LPM_ENABLE
    if (pAppCtxt->isLpmEnabled) {
        /* Disable LPM for 100 ms after any DMA transfers have been completed. */
        Cy_USBD_LpmDisable(pAppCtxt->pUsbdCtxt);
        pAppCtxt->isLpmEnabled  = false;
    }

    pAppCtxt->lpmEnableTime = Cy_USBD_GetTimerTick() + 100;
#endif /* USB3_LPM_ENABLE */
    pAppCtxt->isAppActive   = true;
}

/**************************************************************************************
 * Function: HbDma_Cb
 **************************************************************************************
 * Description: Callback for High BandWidth DMA channels used in the application.
 * Parameters:
 *   handle  : DMA channel handle
 *   type    : Type of callback.
 *   pbufStat: Pointer to DMA buffer status.
 *   userCtx : Callback context structure.
 * Return: void
 *************************************************************************************/
static void HbDma_Cb(
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t* pbufStat,
        void *userCtx)
{
    cy_en_hbdma_mgr_status_t dmaStat;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

#if USE_IP_IP_CHANNEL
    cy_stc_hbdma_buff_status_t bufStat;

    if (type == CY_HBDMA_CB_PROD_EVENT)
    {
        dmaStat = Cy_HBDma_Channel_GetBuffer(handle, &bufStat);
        if (dmaStat == CY_HBDMA_MGR_SUCCESS)
        {
            dmaStat = Cy_HBDma_Channel_CommitBuffer(handle, &bufStat);
            if (dmaStat != CY_HBDMA_MGR_SUCCESS)
            {
                DBG_APP_ERR("CommitBuffer failed: %x\r\n", dmaStat);
            }
        }
        else
        {
            DBG_APP_ERR("GetBuffer failed: %x\r\n", dmaStat);
        }
    }
#else
    LoopBackContext_t     *lpCtxt_p;
    uint32_t dataSize;
    uint8_t i;

    if (type == CY_HBDMA_CB_XFER_CPLT)
    {
        if (handle->type == CY_HBDMA_TYPE_IP_TO_MEM) {

            /* Find the EP pair associated with the channel. */
            for (i = 0; i < pAppCtxt->numLbkPairs; i++) {
                lpCtxt_p = &(pAppCtxt->loopbackInfo[i]);
                if (((uint8_t)handle->prodSckId[0] - CY_HBDMA_USBIN_SOCKET_00) == lpCtxt_p->OutEndpNum) {
                    break;
                }
            }

            /* OUT transfer completed. Find out the size of data received and notify the ECHO thread. */
            dmaStat = Cy_HBDma_Channel_WaitForReceiveCplt(handle, 0, &dataSize);
            if (dmaStat == CY_HBDMA_MGR_SUCCESS) {
                lpCtxt_p->packetLen[lpCtxt_p->nextRdIdx] = (uint16_t)dataSize;
                lpCtxt_p->BulkOutDmaDone = true;
#if FREERTOS_ENABLE
                Cy_USB_AppSignalTask(pAppCtxt, (1U << (i + 1)));
#else
                Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
            } else {
                DBG_APP_ERR("WaitForRecvCplt error %x\r\n", dmaStat);
            }
        } else {

            /* Find the EP pair associated with the channel. */
            for (i = 0; i < pAppCtxt->numLbkPairs; i++) {
                lpCtxt_p = &(pAppCtxt->loopbackInfo[i]);
                if (((uint8_t)handle->consSckId[0] - CY_HBDMA_USBEG_SOCKET_00) == lpCtxt_p->InEndpNum) {
                    break;
                }
            }

            dmaStat = Cy_HBDma_Channel_WaitForSendCplt(handle, 0);
            if (dmaStat == CY_HBDMA_MGR_SUCCESS) {
                lpCtxt_p->BulkInDmaDone = true;
#if FREERTOS_ENABLE
                Cy_USB_AppSignalTask(pAppCtxt, (1U << (i + 1)));
#else
                Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
            } else {
                DBG_APP_ERR("WaitForSendCplt error %x\r\n", dmaStat);
            }
        }
    }
#endif /* USE_IP_IP_CHANNEL */

    Cy_USB_App_KeepLinkActive(pAppCtxt);
}

/**************************************************************************************
 * Function: HbDmaSnk_Cb
 **************************************************************************************
 * Description: Callback for High BandWidth IP-TO-IP sink DMA channel.
 * Parameters:
 *   handle  : DMA channel handle
 *   type    : Type of callback.
 *   pbufStat: Pointer to DMA buffer status.
 *   userCtx : Callback context structure.
 * Return: void
 *************************************************************************************/
static void HbDmaSnk_Cb (
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t* pbufStat,
        void *userCtx)
{
    cy_en_hbdma_mgr_status_t   dmaStat;
    cy_stc_hbdma_buff_status_t bufStat;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    {
        dmaStat = Cy_HBDma_Channel_GetBuffer(handle, &bufStat);
        if (dmaStat == CY_HBDMA_MGR_SUCCESS)
        {
            dmaStat = Cy_HBDma_Channel_DiscardBuffer(handle, &bufStat);
            if (dmaStat != CY_HBDMA_MGR_SUCCESS)
            {
                DBG_APP_ERR("DiscardBuffer failed: %x\r\n", dmaStat);
            }
        }
        else
        {
            DBG_APP_ERR("GetBuffer failed: %x\r\n", dmaStat);
        }
    }

    Cy_USB_App_KeepLinkActive(pAppCtxt);
}

/**************************************************************************************
 * Function: HbDmaSrc_Cb
 **************************************************************************************
 * Description: Callback for High BandWidth IP-TO-IP source DMA channel.
 * Parameters:
 *   handle  : DMA channel handle
 *   type    : Type of callback.
 *   pbufStat: Pointer to DMA buffer status.
 *   userCtx : Callback context structure.
 * Return: void
 *************************************************************************************/
void HbDmaSrc_Cb (
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t* pbufStat,
        void *userCtx)
{
    cy_en_hbdma_mgr_status_t   dmaStat;
    cy_stc_hbdma_buff_status_t bufStat;
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)userCtx;

    {
        dmaStat = Cy_HBDma_Channel_GetBuffer(handle, &bufStat);
        if (dmaStat == CY_HBDMA_MGR_SUCCESS)
        {
            bufStat.count = bufStat.size;
            dmaStat = Cy_HBDma_Channel_CommitBuffer(handle, &bufStat);
            if (dmaStat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("CommitBuffer failed: %x\r\n", dmaStat);
            }
        }
        else
        {
            DBG_APP_ERR("GetBuffer failed: %x\r\n", dmaStat);
        }
    }

    Cy_USB_App_KeepLinkActive(pAppCtxt);
}

static void AbortDmaTransfer (
        cy_stc_usb_app_ctxt_t *pUsbApp,
        cy_en_usb_endp_dir_t   endpDirection,
        uint32_t               endpNumber,
        bool                   chnDestroy)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
#if USE_IP_IP_CHANNEL
    LoopBackContext_t *pLpbkCtxt = NULL;
    uint8_t linkedEp = 0;
#endif /* USE_IP_IP_CHANNEL */

    /* Find the DMA channel associated with the endpoint and reset it. */
    if (endpDirection == CY_USB_ENDP_DIR_IN) {
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);
    } else {
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);
    }

#if USE_IP_IP_CHANNEL
    /* Shared DMA channels are only used for loop-back endpoint pairs. */
    pLpbkCtxt = CyApp_GetLoopbackInfo(pUsbApp, endpDirection, endpNumber);
    if (pLpbkCtxt->LpbkEnable)
    {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            linkedEp = pLpbkCtxt->OutEndpNum;

            /* We will be using the channel structure in the OUT EP DMA Set. */
            pEndpDmaSet = &(pUsbApp->endpOutDma[linkedEp]);
        } else {
            linkedEp = pLpbkCtxt->InEndpNum;
        }
    }
#endif /* USE_IP_IP_CHANNEL */

    Cy_HBDma_Channel_Reset(&(pEndpDmaSet->hbDmaChannel));
    if (chnDestroy) {
        Cy_HBDma_Channel_Destroy(&(pEndpDmaSet->hbDmaChannel));
    }

    /* Flush the socket and reset the endpoint. */
    Cy_USBD_ResetEndp(pUsbApp->pUsbdCtxt, endpNumber, endpDirection, false);
    Cy_SysLib_Delay(1);

#if USE_IP_IP_CHANNEL
    /*
     * If we have linked the OUT and IN endpoints on a single channel, the EP in the other direction
     * needs to be flushed as well.
     */
    if (linkedEp != 0) {
        endpDirection = (endpDirection == CY_USB_ENDP_DIR_OUT) ? CY_USB_ENDP_DIR_IN : CY_USB_ENDP_DIR_OUT;
        Cy_USBD_ResetEndp(pUsbApp->pUsbdCtxt, linkedEp, endpDirection, true);
        Cy_SysLib_Delay(1);
    }
#endif /* USE_IP_IP_CHANNEL */
}

static void RestartDmaTransfer (
        cy_stc_usb_app_ctxt_t *pUsbApp,
        cy_en_usb_endp_dir_t   endpDirection,
        uint32_t               endpNumber)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    cy_en_hbdma_mgr_status_t   apiStat;
    LoopBackContext_t         *pLpbkCtxt;

#if (USE_IP_IP_CHANNEL)
    uint8_t                    linkedEp;
#else
    uint8_t                    lpPairIdx;
#endif /* (USE_IP_IP_CHANNEL) */

    cy_stc_hbdma_buff_status_t bufStat;
    uint32_t *pData;
    uint32_t bi;

    (void)pEndpDmaSet;
    (void)apiStat;

#if (!USE_IP_IP_CHANNEL)
    lpPairIdx = CyApp_GetLoopbackPairIdx(pUsbApp, endpDirection, endpNumber);
#endif /* (!USE_IP_IP_CHANNEL) */
    pLpbkCtxt = CyApp_GetLoopbackInfo(pUsbApp, endpDirection, endpNumber);
    if (endpDirection == CY_USB_ENDP_DIR_IN) {
        pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);

        if (!pLpbkCtxt->LpbkEnable) {
            apiStat = Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);
            DBG_APP_INFO("BulkInChannel %x enable: %x\r\n", endpNumber, apiStat);

            for (bi = 0; bi < pEndpDmaSet->hbDmaChannel.count; bi++) {
                apiStat = Cy_HBDma_Channel_GetBuffer(&(pEndpDmaSet->hbDmaChannel), &bufStat);
                if (apiStat == CY_HBDMA_MGR_SUCCESS) {
                    pData = (uint32_t *)(bufStat.pBuffer);
                    pData[0x0000] = 0x10000000UL;
                    if (pLpbkCtxt->bufferSize > 0x0400U) { pData[0x0100] = 0x11010101UL; }
                    if (pLpbkCtxt->bufferSize > 0x0800U) { pData[0x0200] = 0x12020202UL; }
                    if (pLpbkCtxt->bufferSize > 0x0C00U) { pData[0x0300] = 0x13030303UL; }
                    if (pLpbkCtxt->bufferSize > 0x1000U) { pData[0x0400] = 0x14040404UL; }
                    if (pLpbkCtxt->bufferSize > 0x1400U) { pData[0x0500] = 0x15050505UL; }
                    if (pLpbkCtxt->bufferSize > 0x1800U) { pData[0x0600] = 0x16060606UL; }
                    if (pLpbkCtxt->bufferSize > 0x1C00U) { pData[0x0700] = 0x17070707UL; }
                    if (pLpbkCtxt->bufferSize > 0x2000U) { pData[0x0800] = 0x18080808UL; }
                    if (pLpbkCtxt->bufferSize > 0x2400U) { pData[0x0900] = 0x19090909UL; }
                    if (pLpbkCtxt->bufferSize > 0x2800U) { pData[0x0A00] = 0x1A0A0A0AUL; }
                    if (pLpbkCtxt->bufferSize > 0x2C00U) { pData[0x0B00] = 0x1B0B0B0BUL; }
                    if (pLpbkCtxt->bufferSize > 0x3000U) { pData[0x0C00] = 0x1C0C0C0CUL; }
                    if (pLpbkCtxt->bufferSize > 0x3400U) { pData[0x0D00] = 0x1D0D0D0DUL; }
                    if (pLpbkCtxt->bufferSize > 0x3800U) { pData[0x0E00] = 0x1E0E0E0EUL; }
                    if (pLpbkCtxt->bufferSize > 0x3C00U) { pData[0x0F00] = 0x1F0F0F0FUL; }

                    if (pLpbkCtxt->bufferSize > 0x4000U) { pData[0x1000] = 0x20101010UL; }
                    if (pLpbkCtxt->bufferSize > 0x4400U) { pData[0x1100] = 0x21111111UL; }
                    if (pLpbkCtxt->bufferSize > 0x4800U) { pData[0x1200] = 0x22121212UL; }
                    if (pLpbkCtxt->bufferSize > 0x4C00U) { pData[0x1300] = 0x23131313UL; }
                    if (pLpbkCtxt->bufferSize > 0x5000U) { pData[0x1400] = 0x24141414UL; }
                    if (pLpbkCtxt->bufferSize > 0x5400U) { pData[0x1500] = 0x25151515UL; }
                    if (pLpbkCtxt->bufferSize > 0x5800U) { pData[0x1600] = 0x26161616UL; }
                    if (pLpbkCtxt->bufferSize > 0x5C00U) { pData[0x1700] = 0x27171717UL; }
                    if (pLpbkCtxt->bufferSize > 0x6000U) { pData[0x1800] = 0x28181818UL; }
                    if (pLpbkCtxt->bufferSize > 0x6400U) { pData[0x1900] = 0x29191919UL; }
                    if (pLpbkCtxt->bufferSize > 0x6800U) { pData[0x1A00] = 0x2A1A1A1AUL; }
                    if (pLpbkCtxt->bufferSize > 0x6C00U) { pData[0x1B00] = 0x2B1B1B1BUL; }
                    if (pLpbkCtxt->bufferSize > 0x7000U) { pData[0x1C00] = 0x2C1C1C1CUL; }
                    if (pLpbkCtxt->bufferSize > 0x7400U) { pData[0x1D00] = 0x2D1D1D1DUL; }
                    if (pLpbkCtxt->bufferSize > 0x7800U) { pData[0x1E00] = 0x2E1E1E1EUL; }
                    if (pLpbkCtxt->bufferSize > 0x7C00U) { pData[0x1F00] = 0x2F1F1F1FUL; }

                    bufStat.count = bufStat.size;
                    apiStat = Cy_HBDma_Channel_CommitBuffer(&(pEndpDmaSet->hbDmaChannel), &bufStat);
                    if (apiStat != CY_HBDMA_MGR_SUCCESS) {
                        DBG_APP_WARN("Commit buffer failed\r\n");
                    }
                }
            }

            return;
        }

#if USE_IP_IP_CHANNEL
        linkedEp = pLpbkCtxt->OutEndpNum;
        pEndpDmaSet = &(pUsbApp->endpOutDma[linkedEp]);

        /* Just re-enable the loopback DMA channel. */
        apiStat = Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);
        DBG_APP_INFO("LoopBackChannel %x enable: %x\r\n", endpNumber, apiStat);
#else
        /* If write was pending on the DMA channel, treat it as completed. */
        if (pLpbkCtxt->wrQueued) {
            DBG_APP_INFO("WrFlush: EP-%x\r\n", endpNumber);
            pLpbkCtxt->BulkInDmaDone = true;
#if FREERTOS_ENABLE
            Cy_USB_AppSignalTask(pUsbApp, (1U << (lpPairIdx + 1)));
#else
            Cy_App_MarkEpPairPending(lpPairIdx);
#endif /* FREERTOS_ENABLE */
        }
#endif /* USE_IP_IP_CHANNEL */
    } else {
        pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);

        if (!pLpbkCtxt->LpbkEnable) {
            /* Just re-enable the DMA channel. */
            apiStat = Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);
            DBG_APP_INFO("BulkOutChannel %x enable: %x\r\n", endpNumber, apiStat);
            return;
        }

#if USE_IP_IP_CHANNEL
        /* Just re-enable the loopback DMA channel. */
        apiStat = Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);
        DBG_APP_INFO("LoopBackChannel %x enable: %x\r\n", endpNumber, apiStat);
#else
        /* If read was pending on the DMA channel, treat it as aborted. */
        if (pLpbkCtxt->rdQueued) {
            DBG_APP_INFO("RdAbort: EP-%x\r\n", endpNumber);
            pLpbkCtxt->rdQueued = false;
#if FREERTOS_ENABLE
            Cy_USB_AppSignalTask(pUsbApp, (1U << (lpPairIdx + 1)));
#else
            Cy_App_MarkEpPairPending(lpPairIdx);
#endif /* FREERTOS_ENABLE */
        }
#endif /* USE_IP_IP_CHANNEL */
    }
}

/**************************************************************************************
 * Function: Cy_USB_AppSetupEndpDmaParamsSS
 **************************************************************************************
 * Description: Function to create DMA channels corresponding to OUT/IN endpoints.
 * Parameters:
 *   pUsbApp  : USB application context structure.
 *   pEndpDscr: Pointer to endpoint descriptor.
 *   isUsb2   : Whether current USB connection is USB 2.x.
 * Return: void
 *************************************************************************************/
static void Cy_USB_AppSetupEndpDmaParamsSS (
        cy_stc_usb_app_ctxt_t *pUsbApp,
        uint8_t *pEndpDscr,
        bool isUsb2)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t  mgrStat;
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    uint32_t endpNumber, endpDir, endpType;
    uint16_t maxPktSize;
    LoopBackContext_t *pLpbkCtxt = NULL;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &endpDir);
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    /* Look up the loopback context structure by endpoint number. */
    if (endpDir) {
        pLpbkCtxt = CyApp_GetLoopbackInfo(pUsbApp, CY_USB_ENDP_DIR_IN, endpNumber);
        if (pLpbkCtxt == NULL) {
            DBG_APP_ERR("Failed to find loopback context for IN endpoint %d\r\n", endpNumber);
            return;
        }
    } else {
        pLpbkCtxt = CyApp_GetLoopbackInfo(pUsbApp, CY_USB_ENDP_DIR_OUT, endpNumber);
        if (pLpbkCtxt == NULL) {
            DBG_APP_ERR("Failed to find loopback context for OUT endpoint %d\r\n", endpNumber);
            return;
        }
    }

    if (pLpbkCtxt->LpbkEnable) {
        dmaConfig.size           = pLpbkCtxt->bufferSize;      /* DMA buffer size in bytes */
        dmaConfig.count          = pLpbkCtxt->bufferCnt;       /* Number of DMA buffers per channel. */
        dmaConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
        dmaConfig.prodHdrSize    = 0;                          /* No header to be added. */
        dmaConfig.prodBufSize    = pLpbkCtxt->bufferSize;      /* Same as DMA buffer size as there is no header. */
        dmaConfig.prodSckCount   = 1;                          /* No. of producer sockets */
        dmaConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
        dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
        dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
        dmaConfig.eventEnable    = 0;                          /* No events to be sent. */
        dmaConfig.cb             = HbDma_Cb;                   /* HB-DMA callback */
        dmaConfig.userCtx        = (void *)(pUsbApp);          /* Pass the application context as user context. */

        if (endpDir) {
#if USE_IP_IP_CHANNEL
            DBG_APP_TRACE("Skip egress channel creation %d\r\n", endpNumber);
#else
            pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);

            /*
             * In case of loop-back EP pairs, only allocate one DMA buffer of 1KB on the egress DMA channel.
             * The buffers allocated as part of the ingress channel will be used for holding the data.
             */
            dmaConfig.size  = 1024U;
            dmaConfig.count = 1;

            /* Create channel which will move data from SRAM to USB endpoint. */
            dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;
            dmaConfig.prodSck[0]     = CY_HBDMA_VIRT_SOCKET_WR;
            dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNumber);
            dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_CONSUME_EVENT_Msk |
                USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk;
            mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                    &(pEndpDmaSet->hbDmaChannel),
                    &dmaConfig);

            if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("BulkIn channel create failed 0x%x\r\n", mgrStat);
                return;
            } else {
                /* Make sure endpoint is marked valid. */
                DBG_APP_INFO("HBDma channel created for EP%d-In\r\n", endpNumber);
                pEndpDmaSet->maxPktSize = maxPktSize;
                pEndpDmaSet->valid      = 0x01;
            }
#endif /* USE_IP_IP_CHANNEL */
        } else {
            pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);

#if USE_IP_IP_CHANNEL
            /*
             * Look up the IN endpoint corresponding to this OUT endpoint and
             * create the channel using the corresponding socket.
             */
            uint8_t inEndp = pLpbkCtxt->InEndpNum;

            dmaConfig.chType = CY_HBDMA_TYPE_IP_TO_IP;
            dmaConfig.prodSck[0] = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + endpNumber);
            dmaConfig.consSck[0] = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + inEndp);

            if (pUsbApp->autoDmaEnable) {
                dmaConfig.eventEnable = true;
                dmaConfig.intrEnable  = 0;
            } else {
                dmaConfig.eventEnable = false;
                dmaConfig.intrEnable  = (USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_PRODUCE_EVENT_Msk |
                        USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_CONSUME_EVENT_Msk);
            }

            mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                    &(pEndpDmaSet->hbDmaChannel), &dmaConfig);
            if (mgrStat == CY_HBDMA_MGR_SUCCESS) {
                if (!isUsb2) {
                    /* When an OUT endpoint has a max. packet size which is not a power of 2, we need
                     * to program the number of packets that can fit in one DMA buffer.
                     */
                    if ((maxPktSize & (maxPktSize - 1)) != 0) {
                        Cy_USBD_EndpSetPktsPerBuffer(pUsbApp->pUsbdCtxt, endpNumber,
                                (dmaConfig.size / maxPktSize));
                    }

                    if ((pUsbApp->burstModeEnable) && (endpType == CY_USB_ENDP_TYPE_ISO)) {
                        DBG_APP_TRACE("BURSTMODE enable: %d %d\r\n", endpNumber, inEndp);
                        Cy_USBD_SetEpBurstMode(pUsbApp->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
                        Cy_USBD_SetEpBurstMode(pUsbApp->pUsbdCtxt, inEndp, CY_USB_ENDP_DIR_IN, true);
                    } else {
                        DBG_APP_TRACE("BURSTMODE disable: %d %d\r\n", endpNumber, inEndp);
                        Cy_USBD_SetEpBurstMode(pUsbApp->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, false);
                        Cy_USBD_SetEpBurstMode(pUsbApp->pUsbdCtxt, inEndp, CY_USB_ENDP_DIR_IN, false);
                    }

                    mgrStat = Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);
                    DBG_APP_TRACE("Channel %d enable status=%x\r\n", endpNumber, mgrStat);
                    pEndpDmaSet->maxPktSize = maxPktSize;
                    pEndpDmaSet->valid      = 0x01;
                }

                /* Retrieve and store the DMA buffer pointers for both ingress and egress transfers. */
                Cy_HBDma_Channel_GetBufferInfo(&(pEndpDmaSet->hbDmaChannel),
                        pLpbkCtxt->outEpBuf, dmaConfig.count);
                Cy_HBDma_Channel_GetBufferInfo(&(pEndpDmaSet->hbDmaChannel),
                        pLpbkCtxt->inEpBuf, dmaConfig.count);
            } else {
                DBG_APP_ERR("Channel %d create status=%x\r\n", endpNumber, mgrStat);
            }
#else
            /* Create channel which moves data from USB ingress endpoint into HBW SRAM. */
            dmaConfig.chType         = CY_HBDMA_TYPE_IP_TO_MEM;
            dmaConfig.prodSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + endpNumber);
            dmaConfig.consSck[0]     = CY_HBDMA_VIRT_SOCKET_RD;
            dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_PRODUCE_EVENT_Msk |
                USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_TRANS_DONE_Msk;
            mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                    &(pEndpDmaSet->hbDmaChannel),
                    &dmaConfig);
            if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("BulkOut channel create failed 0x%x\r\n", mgrStat);
                return;
            } else {
                /* Make sure endpoint is marked valid. */
                DBG_APP_INFO("HBDma channel created for EP%d-Out\r\n", endpNumber);
                pEndpDmaSet->maxPktSize = maxPktSize;
                pEndpDmaSet->valid      = 0x01;

                /* Retrieve and store the DMA buffer pointers for both ingress and egress transfers. */
                Cy_HBDma_Channel_GetBufferInfo(&(pEndpDmaSet->hbDmaChannel),
                        pLpbkCtxt->outEpBuf, dmaConfig.count);
                Cy_HBDma_Channel_GetBufferInfo(&(pEndpDmaSet->hbDmaChannel),
                        pLpbkCtxt->inEpBuf, dmaConfig.count);

                if (!isUsb2) {
                    /* When an OUT endpoint has a max. packet size which is not a power of 2, we need
                     * to program the number of packets that can fit in one DMA buffer.
                     */
                    if ((maxPktSize & (maxPktSize - 1)) != 0) {
                        Cy_USBD_EndpSetPktsPerBuffer(pUsbApp->pUsbdCtxt, endpNumber,
                                (dmaConfig.size / maxPktSize));
                    }
                }
            }
#endif /* USE_IP_IP_CHANNEL */
        }
    } else {
        if (endpDir == 0) {
            /* Create and enable a IP-to-MEM data channel. */
            dmaConfig.chType         = CY_HBDMA_TYPE_IP_TO_MEM;
            dmaConfig.size           = pLpbkCtxt->bufferSize;      /* DMA buffer size in bytes */
            dmaConfig.count          = pLpbkCtxt->bufferCnt;       /* Number of DMA buffers per channel. */
            dmaConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
            dmaConfig.prodHdrSize    = 0;                          /* No header to be added. */
            dmaConfig.prodBufSize    = pLpbkCtxt->bufferSize;      /* Same as DMA buffer size as there is no header. */
            dmaConfig.prodSckCount   = 1;                          /* No. of producer sockets */
            dmaConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
            dmaConfig.prodSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBIN_SOCKET_00 + endpNumber);
            dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
            dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
            dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
            dmaConfig.eventEnable    = 0;                          /* No events to be sent. */
            dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_PRODUCE_EVENT_Msk;
            dmaConfig.cb             = HbDmaSnk_Cb;                /* HB-DMA callback */
            dmaConfig.userCtx        = (void *)(pUsbApp);          /* Pass the application context as user context. */

            pEndpDmaSet = &(pUsbApp->endpOutDma[endpNumber]);
            mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                    &(pEndpDmaSet->hbDmaChannel),
                    &dmaConfig);
            if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("IpToMem channel create failed 0x%x\r\n", mgrStat);
                return;
            } else {
                /* Retrieve and store the DMA buffer pointers. */
                Cy_HBDma_Channel_GetBufferInfo(&(pEndpDmaSet->hbDmaChannel),
                        pLpbkCtxt->outEpBuf, dmaConfig.count);

                if (!isUsb2) {
                    /* Enable burst mode for the streaming IN/OUT endpoints. */
                    Cy_USBD_SetEpBurstMode(pUsbApp->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);

                    /* Make sure endpoint is marked valid. */
                    DBG_APP_INFO("IPToMem channel created for EP%d\r\n", endpNumber);
                    pEndpDmaSet->maxPktSize = maxPktSize;
                    pEndpDmaSet->valid      = 0x01;

                    /* When an OUT endpoint has a max. packet size which is not a power of 2, we need
                     * to program the number of packets that can fit in one DMA buffer.
                     */
                    if ((maxPktSize & (maxPktSize - 1)) != 0) {
                        Cy_USBD_EndpSetPktsPerBuffer(pUsbApp->pUsbdCtxt, endpNumber,
                                (dmaConfig.size / maxPktSize));
                    }

                    Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);
                }
            }
        } else {
            /* Create and enable a MEM-to-IP data channel. */
            dmaConfig.chType         = CY_HBDMA_TYPE_MEM_TO_IP;
            dmaConfig.size           = pLpbkCtxt->bufferSize;      /* DMA buffer size in bytes. */
            dmaConfig.count          = pLpbkCtxt->bufferCnt;       /* Number of DMA buffers per channel. */
            dmaConfig.bufferMode     = false;                      /* DMA buffer mode disabled */
            dmaConfig.prodHdrSize    = 0;                          /* No header to be added. */
            dmaConfig.prodBufSize    = pLpbkCtxt->bufferSize;      /* Same as DMA buffer size as there is no header. */
            dmaConfig.prodSckCount   = 1;                          /* No. of producer sockets */
            dmaConfig.consSckCount   = 1;                          /* No. of consumer Sockets */
            dmaConfig.prodSck[0]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
            dmaConfig.prodSck[1]     = (cy_hbdma_socket_id_t)0;    /* Producer Socket ID: None */
            dmaConfig.consSck[0]     = (cy_hbdma_socket_id_t)(CY_HBDMA_USBEG_SOCKET_00 + endpNumber);
            dmaConfig.consSck[1]     = (cy_hbdma_socket_id_t)0;    /* Consumer Socket ID: None */
            dmaConfig.eventEnable    = 0;                          /* No events to be sent. */
            dmaConfig.intrEnable     = USB32DEV_ADAPTER_DMA_SCK_INTR_MASK_CONSUME_EVENT_Msk;
            dmaConfig.cb             = HbDmaSrc_Cb;                /* HB-DMA callback */
            dmaConfig.userCtx        = (void *)(pUsbApp);          /* Pass the application context as user context. */
            dmaConfig.endpAddr       = endpNumber;

            pEndpDmaSet = &(pUsbApp->endpInDma[endpNumber]);
            mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                    &(pEndpDmaSet->hbDmaChannel),
                    &dmaConfig);
            if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("MemToIp channel create failed 0x%x\r\n", mgrStat);
                return;
            } else {
                /* Retrieve and store the DMA buffer pointers. */
                Cy_HBDma_Channel_GetBufferInfo(&(pEndpDmaSet->hbDmaChannel),
                        pLpbkCtxt->inEpBuf, dmaConfig.count);

                if (!isUsb2) {
                    /* Enable burst mode for the streaming IN/OUT endpoints. */
                    Cy_USBD_SetEpBurstMode(pUsbApp->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_IN, true);

                    /* Make sure endpoint is marked valid. */
                    DBG_APP_INFO("MemToIp channel created for EP%d\r\n", endpNumber);
                    pEndpDmaSet->maxPktSize = maxPktSize;
                    pEndpDmaSet->valid      = 0x01;

                    Cy_HBDma_Channel_Enable(&(pEndpDmaSet->hbDmaChannel), 0);

                    uint32_t bi;
                    for (bi = 0; bi < dmaConfig.count; bi++)
                    {
                        cy_stc_hbdma_buff_status_t bufStat;
                        cy_en_hbdma_mgr_status_t apiStat;
                        uint32_t *pData;

                        apiStat = Cy_HBDma_Channel_GetBuffer(&(pEndpDmaSet->hbDmaChannel), &bufStat);
                        if (apiStat == CY_HBDMA_MGR_SUCCESS)
                        {
                            pData = (uint32_t *)(bufStat.pBuffer);

                            /* Zero out the buffer first and then update the first bytes in each 1KB packet. */
                            Cy_UsbFx_MemSetDword(pData, 0, pLpbkCtxt->bufferSize);

                            pData[0x0000] = 0x10000000UL;
                            if (pLpbkCtxt->bufferSize > 0x0400U) { pData[0x0100] = 0x11010101UL; }
                            if (pLpbkCtxt->bufferSize > 0x0800U) { pData[0x0200] = 0x12020202UL; }
                            if (pLpbkCtxt->bufferSize > 0x0C00U) { pData[0x0300] = 0x13030303UL; }
                            if (pLpbkCtxt->bufferSize > 0x1000U) { pData[0x0400] = 0x14040404UL; }
                            if (pLpbkCtxt->bufferSize > 0x1400U) { pData[0x0500] = 0x15050505UL; }
                            if (pLpbkCtxt->bufferSize > 0x1800U) { pData[0x0600] = 0x16060606UL; }
                            if (pLpbkCtxt->bufferSize > 0x1C00U) { pData[0x0700] = 0x17070707UL; }
                            if (pLpbkCtxt->bufferSize > 0x2000U) { pData[0x0800] = 0x18080808UL; }
                            if (pLpbkCtxt->bufferSize > 0x2400U) { pData[0x0900] = 0x19090909UL; }
                            if (pLpbkCtxt->bufferSize > 0x2800U) { pData[0x0A00] = 0x1A0A0A0AUL; }
                            if (pLpbkCtxt->bufferSize > 0x2C00U) { pData[0x0B00] = 0x1B0B0B0BUL; }
                            if (pLpbkCtxt->bufferSize > 0x3000U) { pData[0x0C00] = 0x1C0C0C0CUL; }
                            if (pLpbkCtxt->bufferSize > 0x3400U) { pData[0x0D00] = 0x1D0D0D0DUL; }
                            if (pLpbkCtxt->bufferSize > 0x3800U) { pData[0x0E00] = 0x1E0E0E0EUL; }
                            if (pLpbkCtxt->bufferSize > 0x3C00U) { pData[0x0F00] = 0x1F0F0F0FUL; }

                            if (pLpbkCtxt->bufferSize > 0x4000U) { pData[0x1000] = 0x20101010UL; }
                            if (pLpbkCtxt->bufferSize > 0x4400U) { pData[0x1100] = 0x21111111UL; }
                            if (pLpbkCtxt->bufferSize > 0x4800U) { pData[0x1200] = 0x22121212UL; }
                            if (pLpbkCtxt->bufferSize > 0x4C00U) { pData[0x1300] = 0x23131313UL; }
                            if (pLpbkCtxt->bufferSize > 0x5000U) { pData[0x1400] = 0x24141414UL; }
                            if (pLpbkCtxt->bufferSize > 0x5400U) { pData[0x1500] = 0x25151515UL; }
                            if (pLpbkCtxt->bufferSize > 0x5800U) { pData[0x1600] = 0x26161616UL; }
                            if (pLpbkCtxt->bufferSize > 0x5C00U) { pData[0x1700] = 0x27171717UL; }
                            if (pLpbkCtxt->bufferSize > 0x6000U) { pData[0x1800] = 0x28181818UL; }
                            if (pLpbkCtxt->bufferSize > 0x6400U) { pData[0x1900] = 0x29191919UL; }
                            if (pLpbkCtxt->bufferSize > 0x6800U) { pData[0x1A00] = 0x2A1A1A1AUL; }
                            if (pLpbkCtxt->bufferSize > 0x6C00U) { pData[0x1B00] = 0x2B1B1B1BUL; }
                            if (pLpbkCtxt->bufferSize > 0x7000U) { pData[0x1C00] = 0x2C1C1C1CUL; }
                            if (pLpbkCtxt->bufferSize > 0x7400U) { pData[0x1D00] = 0x2D1D1D1DUL; }
                            if (pLpbkCtxt->bufferSize > 0x7800U) { pData[0x1E00] = 0x2E1E1E1EUL; }
                            if (pLpbkCtxt->bufferSize > 0x7C00U) { pData[0x1F00] = 0x2F1F1F1FUL; }

                            bufStat.count = bufStat.size;
                            apiStat = Cy_HBDma_Channel_CommitBuffer(&(pEndpDmaSet->hbDmaChannel), &bufStat);
                        }

                        if (apiStat != CY_HBDMA_MGR_SUCCESS)
                        {
                            break;
                        }
                    }
                }
            }
        }
    }
}

/*
 * Function: Cy_USB_AppSetupEndpDmaParamsHS()
 * Description: This Function will setup Endpoint and DMA related parameters
 *              before transfer initiated.
 * Parameter: cy_stc_usb_app_ctxt_t, pEndpDscr
 * return: void
 */
static void
Cy_USB_AppSetupEndpDmaParamsHS (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr)
{
    cy_stc_app_endp_dma_set_t *pEndpDmaSet;
    DW_Type *pDW;
    uint32_t endpNumber;
    bool stat;
    uint16_t maxPktSize = 0x00;
    cy_en_usb_endp_dir_t endpDirection;

    endpNumber = ((*(pEndpDscr+CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);
    Cy_USBD_GetEndpMaxPktSize(pEndpDscr, &maxPktSize);

    if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80) {
        endpDirection = CY_USB_ENDP_DIR_IN;
        pEndpDmaSet   = &(pUsbApp->endpInDma[endpNumber]);
        pDW           = pUsbApp->pCpuDw1Base;
    } else {
        endpDirection = CY_USB_ENDP_DIR_OUT;
        pEndpDmaSet   = &(pUsbApp->endpOutDma[endpNumber]);
        pDW           = pUsbApp->pCpuDw0Base;
    }

    if (pUsbApp->numLbkPairs <= 2) {
        stat = Cy_USBHS_App_EnableEpDmaCSet(pEndpDmaSet, pUsbApp->pCpuDmacBase,
                ((endpDirection == CY_USB_ENDP_DIR_IN) ? (1U + (endpNumber << 1U)) : (endpNumber << 1U)),
                endpNumber, endpDirection, maxPktSize);
    } else {
        stat = Cy_USBHS_App_EnableEpDmaSet(pEndpDmaSet, pDW, endpNumber, endpNumber, endpDirection, maxPktSize);
    }
    DBG_APP_INFO("Enable EPDmaSet: endp=%x dir=%x stat=%x\r\n", endpNumber, endpDirection, stat);
}   /* end of function  */

/*
 * Function: Cy_USB_AppSetupEndpDmaParams()
 * Description: This Function will setup Endpoint and DMA related parameters
 *              before transfer initiated.
 * Parameter: cy_stc_usb_app_ctxt_t, pEndpDscr
 * return: void
 */
void
Cy_USB_AppSetupEndpDmaParams (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr)
{
    if (pUsbApp->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        Cy_USB_AppSetupEndpDmaParamsSS(pUsbApp, pEndpDscr, false);
    } else {
        /* Call SS DMA setup function to create the DMA channels so that buffers are allocated. */
        Cy_USB_AppSetupEndpDmaParamsSS(pUsbApp, pEndpDscr, true);
        Cy_USB_AppSetupEndpDmaParamsHS(pUsbApp, pEndpDscr);
    }
}

/*
 * Function: Cy_USB_AppConfigureEndp()
 * Description: This Function is used by application to configure endpoints
 *              after set configuration.  This function should be used for
 *              all endpoints except endp0.
 * Parameter: cy_stc_usb_usbd_ctxt_t, pEndpDscr
 * return: void
 */
void
Cy_USB_AppConfigureEndp (cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint8_t isoPkts = 0x00;
    uint8_t burstSize = 0x00;
    uint8_t maxStream = 0x00;
    uint8_t interval = 0x00;
    uint8_t *pCompDscr = NULL;
    cy_en_usbd_ret_code_t usbdRetCode;

    /* If it is not endpoint descriptor then return */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr)) {
        return;
    }
    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);
    
    if (dir) {
        endpDirection = CY_USB_ENDP_DIR_IN;
    } else {
        endpDirection = CY_USB_ENDP_DIR_OUT;
    }
    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType)) {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value. */
        isoPkts = (
                (*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK)
                >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        /* Get companion descriptor and from there get burstSize. */
        pCompDscr = Cy_USBD_GetSsEndpCompDscr(pUsbdCtxt, pEndpDscr);
        Cy_USBD_GetEndpCompnMaxburst(pCompDscr, &burstSize);
        Cy_USBD_GetEndpCompnMaxStream(pCompDscr, &maxStream);
        if (CY_USB_ENDP_TYPE_ISO == endpType) {
            Cy_USBD_GetEndpCompnAttribute(pCompDscr, &isoPkts);
            isoPkts = (isoPkts & 0x03U) + 0x01U;
            isoPkts *= burstSize;

            /* Fetch the endpoint service interval. */
            Cy_USBD_GetEndpInterval(pEndpDscr, &interval);
        }
    }

    /* Prepare endpointConfig parameter. */
    endpConfig.endpType = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection = endpDirection;
    endpConfig.valid = valid;
    endpConfig.endpNumber = endpNumber;
    endpConfig.maxPktSize = (uint32_t)maxPktSize;
    endpConfig.isoPkts = (uint32_t)isoPkts;
    endpConfig.burstSize = burstSize;
    endpConfig.streamID = maxStream;
    endpConfig.interval = interval;
    endpConfig.allowNakTillDmaRdy = (endpConfig.endpType != CY_USB_ENDP_TYPE_ISO);
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug. */
    DBG_APP_INFO("EPCFG:%d %x\r\n", endpNumber, (uint8_t)usbdRetCode);

    return;
}   /* end of function */

/*
 * Function: Cy_USB_AppSignalTask()
 * Description: This Function signals the application task when there is any work for it to perform.
 * Parameter: pAppCtxt, evMask
 * return: Whether yield is required on exit from ISR.
 */
bool
Cy_USB_AppSignalTask(cy_stc_usb_app_ctxt_t *pAppCtxt, const EventBits_t evMask)
{
#if FREERTOS_ENABLE
    BaseType_t wakeTask = pdFALSE;

    xEventGroupSetBitsFromISR(pAppCtxt->appEvGrpHandle, evMask, &wakeTask);
    return (wakeTask != pdFALSE);
#else
    (void)pAppCtxt;
    (void)evMask;
    return false;
#endif /* FREERTOS_ENABLE */
}

/*
 * Function: Cy_USB_AppSetCfgCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSetCfgCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    DBG_APP_INFO("SetCfgCb:Start Speed=%d\r\n", pUsbdCtxt->devSpeed);

    /* Print out the contents of the USB event log buffer for SET_CONFIG request is ACKed. */
    AppPrintUsbEventLog(pUsbApp, pUsbApp->pUsbdCtxt->pSsCalCtxt);

    /* Save the configuration number. */
    pUsbApp->activeCfgNum = pUsbdCtxt->activeCfgNum;

    /* Destroy any active DMA channels. */
    Cy_USB_AppDisableEndpDma(pUsbApp);

    /* Enable the DataWire DMA modules. */
    Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
    Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

    /* Disable clock control during EP reset while endpoints are being configured. */
    Cy_USBSS_Cal_ClkStopOnEpRstEnable(pUsbdCtxt->pSsCalCtxt, false);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg) {
        /* Set config should be called when active config value > 0x00. */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00) {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++) {
        /* During Set Config command always altSetting 0 will be active. */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL) {
            DBG_APP_ERR("NULL intf dscr\r\n");
            return;
        }
        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);

        if (numOfEndp == 0x00) {
            DBG_APP_WARN("numOfEndp is 0\r\n");
            continue;
        }
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            /* Do not try to configure any endpoints beyond the supported range. */
            if (
                    ((pEndpDscr[2] & 0x7F) <= pUsbApp->maxLbkPairs)
               ) {
                Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
                Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
            }

            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                /* Skip over SS companion descriptor. */
                pEndpDscr += 6u;
            }
        }
    }

    if (pUsbdCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
        Cy_USBSS_Cal_PostEpEnable(pUsbdCtxt->pSsCalCtxt);
    }

    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

#if FREERTOS_ENABLE
    /* Signal the task that device state has changed. */
    Cy_USB_AppSignalTask(pUsbApp, EV_DEVSTATE_CHG);
#endif /* FREERTOS_ENABLE */

    DBG_APP_INFO("SetCfgCb:End\r\n");

#if USB3_LPM_ENABLE
    /* Schedule LPM enable after 80 milliseconds. */
    pUsbApp->isLpmEnabled  = false;
    pUsbApp->lpmEnableTime = Cy_USBD_GetTimerTick() + 80;
#else
    DBG_APP_INFO("Disabling LPM transitions\r\n");
    Cy_USBD_LpmDisable(pUsbApp->pUsbdCtxt);
#endif /* USB3_LPM_ENABLE */
}   /* end of function */

/*
 * Function: Cy_USB_AppDisableEndpDma()
 * Description: This function de-inits all active USB DMA channels as part of USB disconnect process.
 * Parameter: cy_stc_usb_app_ctxt_t *
 * return: void
 */
void
Cy_USB_AppDisableEndpDma (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t i;

    if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
        /* If any DMA channels have been created, disable and destroy them. */
        for (i = 0; i < CY_USB_MAX_ENDP_NUMBER; i++) {
            if (pAppCtxt->endpInDma[i].valid) {
                DBG_APP_INFO("HBDMA destroy EP%d-In\r\n", i);
                Cy_HBDma_Channel_Disable(&(pAppCtxt->endpInDma[i].hbDmaChannel));
                Cy_HBDma_Channel_Destroy(&(pAppCtxt->endpInDma[i].hbDmaChannel));
                Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, i, CY_USB_ENDP_DIR_IN);
                Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, i, CY_USB_ENDP_DIR_IN, false);
                pAppCtxt->endpInDma[i].valid = false;
            }

            if (pAppCtxt->endpOutDma[i].valid) {
                DBG_APP_INFO("HBDMA destroy EP%d-Out\r\n", i);
                Cy_HBDma_Channel_Disable(&(pAppCtxt->endpOutDma[i].hbDmaChannel));
                Cy_HBDma_Channel_Destroy(&(pAppCtxt->endpOutDma[i].hbDmaChannel));
                Cy_USBD_FlushEndp(pAppCtxt->pUsbdCtxt, i, CY_USB_ENDP_DIR_OUT);
                Cy_USBD_ResetEndp(pAppCtxt->pUsbdCtxt, i, CY_USB_ENDP_DIR_OUT, false);
                pAppCtxt->endpOutDma[i].valid = false;
            }
        }
    } else {
        for (i = 1; i < CY_USB_MAX_ENDP_NUMBER; i++) {
            if (pAppCtxt->endpInDma[i].valid) {
                /* Make sure the DMA channel is destroyed so that all memory is freed up. */
                Cy_HBDma_Channel_Destroy(&(pAppCtxt->endpInDma[i].hbDmaChannel));

                /* DeInit the DMA channel and disconnect the triggers. */
                if (pAppCtxt->numLbkPairs <= 2) {
                    Cy_USBHS_App_DisableEpDmaCSet(&(pAppCtxt->endpInDma[i]));
                } else {
                    Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpInDma[i]));
                }
            }

            if (pAppCtxt->endpOutDma[i].valid) {
                /* Make sure the DMA channel is destroyed so that all memory is freed up. */
                Cy_HBDma_Channel_Destroy(&(pAppCtxt->endpOutDma[i].hbDmaChannel));

                /* DeInit the DMA channel and disconnect the triggers. */
                if (pAppCtxt->numLbkPairs <= 2) {
                    Cy_USBHS_App_DisableEpDmaCSet(&(pAppCtxt->endpOutDma[i]));
                } else {
                    Cy_USBHS_App_DisableEpDmaSet(&(pAppCtxt->endpOutDma[i]));
                }
            }
        }
    }
}

/*
 * Function: Cy_USB_AppBusResetCallback()
 * Description: This Function will be called by USBD when bus detects RESET.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppBusResetCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    Cy_USB_AppDisableEndpDma(pUsbApp);

    /*
     * As off now USBD layer should take care of itself and CAL layer
     * and application layer should reset it's data structure.
     * TBD: Any change in above assumption will require review and
     *      updated code.
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase,
                   pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base, pUsbApp->pHbDmaMgr);
    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->activeCfgNum = 0;

#if FREERTOS_ENABLE
    /* Signal the task that device state has changed. */
    Cy_USB_AppSignalTask(pUsbApp, EV_DEVSTATE_CHG);
#endif /* FREERTOS_ENABLE */

    /* Keep U1/U2 transitions disabled at the start of connection. */
    Cy_USBD_LpmDisable(pUsbApp->pUsbdCtxt);

    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppBusResetDoneCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppBusResetDoneCallback (void *pAppCtxt,
                                cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;

#if FREERTOS_ENABLE
    /* Signal the task that device state has changed. */
    Cy_USB_AppSignalTask(pUsbApp, EV_DEVSTATE_CHG);
#endif /* FREERTOS_ENABLE */

    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppBusSpeedCallback()
 * Description: This Function will be called by USBD  layer when
 *              speed is identified or speed change is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppBusSpeedCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                            cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

#if FREERTOS_ENABLE
    /* Signal the task that device state has changed. */
    Cy_USB_AppSignalTask(pUsbApp, EV_DEVSTATE_CHG);
#endif /* FREERTOS_ENABLE */

    return;
}   /* end of function. */

static void Cy_USB_AppHandleEpReset (cy_stc_usb_app_ctxt_t *pUsbApp,
        uint8_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    LoopBackContext_t *pLpbkCtxt = NULL;
    uint8_t lpPairIdx;

    /*
     * Handling of CLEAR_FEATURE(EP_HALT) requires the following steps:
     * 1. Disable any ongoing DMA transfer
     * 2. Flush, reset and clear STALL on the endpoint.
     * 3. Clear loop-back state so that DMA transfer will be restarted.
     */
    Cy_USB_AppTerminateDma(pUsbApp, endpNumber, endpDirection);
    Cy_USBD_FlushEndp(pUsbApp->pUsbdCtxt, endpNumber, endpDirection);
    Cy_USBD_ResetEndp(pUsbApp->pUsbdCtxt, endpNumber, endpDirection, false);
    Cy_USB_USBD_EndpSetClearStall(pUsbApp->pUsbdCtxt, endpNumber, endpDirection, false);

    if (endpDirection == CY_USB_ENDP_DIR_IN) {

        /* Look up the ep pair index corresponding to the endpoint. */
        for (lpPairIdx = 0; lpPairIdx < pUsbApp->numLbkPairs; lpPairIdx++) {
            if (pUsbApp->loopbackInfo[lpPairIdx].InEndpNum == endpNumber) {
                pLpbkCtxt = &(pUsbApp->loopbackInfo[lpPairIdx]);
                break;
            }
        }

        /* If write was pending on the DMA channel, treat it as completed. */
        if ((pLpbkCtxt != NULL) && (pLpbkCtxt->wrQueued)) {
            DBG_APP_INFO("WrFlush: EP-%x\r\n", endpNumber);
            pLpbkCtxt->BulkInDmaDone = true;

#if FREERTOS_ENABLE
            Cy_USB_AppSignalTask(pUsbApp, (1U << (lpPairIdx + 1)));
#else
            Cy_App_MarkEpPairPending(lpPairIdx);
#endif /* FREERTOS_ENABLE */
        }
    } else {

        /* Look up the ep pair index corresponding to the endpoint. */
        for (lpPairIdx = 0; lpPairIdx < pUsbApp->numLbkPairs; lpPairIdx++) {
            if (pUsbApp->loopbackInfo[lpPairIdx].OutEndpNum == endpNumber) {
                pLpbkCtxt = &(pUsbApp->loopbackInfo[lpPairIdx]);
                break;
            }
        }

        /* If read was pending on the DMA channel, treat it as aborted. */
        if ((pLpbkCtxt != NULL) && (pLpbkCtxt->rdQueued)) {
            DBG_APP_INFO("RdAbort: EP-%x\r\n", endpNumber);
            pLpbkCtxt->rdQueued = false;

#if FREERTOS_ENABLE
            Cy_USB_AppSignalTask(pUsbApp, (1U << (lpPairIdx + 1)));
#else
            Cy_App_MarkEpPairPending(lpPairIdx);
#endif /* FREERTOS_ENABLE */
        }
    }
}

/*
 * Function: Cy_USB_AppEp0RecvCallback()
 * Description: This Function will be called by USBD layer when
 *              an EP0-OUT data transfer is complete.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppEp0RecvCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
    if (SetSelRqtPending) {
        SetSelRqtPending = false;
        DBG_APP_INFO("SET_SEL request complete: %x %x\r\n",
                SetSelDataBuffer[0], SetSelDataBuffer[1]);
    } else {
        DBG_APP_TRACE("EP0 Receive done: %x %x\r\n", pMsg->data[0], pMsg->data[1]);
    }
}

extern volatile bool VendorCtrlRqtPending;
extern uint8_t CyFxUSBReportDscr[];

/*
 * Function: Cy_USB_AppSetupCallback()
 * Description: This Function will be called by USBD  layer when
 *              set configuration command successful. This function
 *              does sanity check and prepare device for function
 *              to take over.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSetupCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                         cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    bool isReqHandled = false;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;

    uint8_t bRequest, bReqType, bTarget;
    uint16_t wLength, wValue, wIndex;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;

    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bTarget = (bReqType & CY_USB_CTRL_REQ_RECIPENT_MASK);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wLength  = pUsbdCtxt->setupReq.wLength;
    wValue   = pUsbdCtxt->setupReq.wValue;
    wIndex = pUsbdCtxt->setupReq.wIndex;

    /* If trying to bind to WinUSB driver, we need to support additional control requests. */
    if (pUsbApp->winusbEnabled)
    {
        /* Handle Microsoft OS String Descriptor request. */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

            /* Make sure we do not send more data than requested. */
            if (wLength > glOsString[0]) {
                wLength = glOsString[0];
            }

            DBG_APP_INFO("OSString\r\n");
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsString, wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS) {
                isReqHandled = true;
            }
        }

        /* Handle OS Compatibility and OS Feature requests */
        if (bRequest == MS_VENDOR_CODE) {
            if (wIndex == 0x04) {

                if (wLength > *((uint16_t *)glOsCompatibilityId)) {
                    wLength = *((uint16_t *)glOsCompatibilityId);
                }

                DBG_APP_INFO("OSCompat\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsCompatibilityId, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
            else if (wIndex == 0x05) {

                if (wLength > *((uint16_t *)glOsFeature)) {
                    wLength = *((uint16_t *)glOsFeature);
                }

                DBG_APP_INFO("OSFeature\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsFeature, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
        }

        if (isReqHandled) {
            return;
        }
    }

    /* SET_SEL request is supposed to have an OUT data phase of 6 bytes. */
    if ((bRequest == CY_USB_SC_SET_SEL) && (wLength == 6)) {
        retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)SetSelDataBuffer, wLength);
        if (retStatus != CY_USBD_STATUS_SUCCESS) {
            DBG_APP_INFO("SET_SEL Recv error: %d\r\n", retStatus);
        } else {
            /* RecvEndp0Data function is non-blocking. Remember that request is pending. */
            SetSelRqtPending = true;
            isReqHandled = true;
        }
    }

    if (((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS) == CY_USB_CTRL_REQ_VENDOR)
    {
        /* In USBSS connection, disable LPM entry for the next 100 ms. */
        if (pUsbApp->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            Cy_USB_App_KeepLinkActive(pUsbApp);
        }

#if FREERTOS_ENABLE
        Cy_USB_AppSignalTask(pUsbApp, EV_VENDOR_REQUEST);
#else
        VendorCtrlRqtPending = true;
#endif /* FREERTOS_ENABLE */

        /* Vendor commands to be handled in task thread. */
        isReqHandled = true;
    }

    if (bRequest == CY_USB_SC_SET_FEATURE) {
        if (
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)
           ) {
            epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL), epDir, true);
            Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
            isReqHandled = true;
        }

        if (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) {
            if (wValue == CY_USB_FEATURE_DEVICE_REMOTE_WAKE) {
                /* Set Remote Wakeup enable: ACK the request. State is tracked by stack. */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }

            if ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE)) {
                /* Set U1/U2 enable. ACK the request. Enable LPM if allowed by configuration. */
#if USB3_LPM_ENABLE
                if (pUsbApp->isLpmEnabled == false) {
                    DBG_APP_INFO("Enabling LPM\r\n");
                    pUsbApp->isLpmEnabled = true;
                    Cy_USBD_LpmEnable(pUsbdCtxt);
                }
#endif /* USB3_LPM_ENABLE */

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == CY_USB_FEATURE_FUNC_SUSPEND)) {
            /* We only support one interface. */
            if ((wIndex & 0xFFU) == 0) {
                if ((wIndex & 0x0200U) != 0) {
                    DBG_APP_INFO("Enable FnWake\r\n");
                    pUsbApp->functionWakeEnable = true;
                } else {
                    DBG_APP_INFO("Disable FnWake\r\n");
                    pUsbApp->functionWakeEnable = false;
                    pUsbApp->functionWakeTimestamp = 0;
                }
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }
    }

    if (bRequest == CY_USB_SC_CLEAR_FEATURE) {
        if (
                (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
                (wValue == CY_USB_FEATURE_ENDP_HALT)
           ) {
#if FREERTOS_ENABLE
            Cy_USB_AppSignalTask(pUsbApp, EV_VENDOR_REQUEST);
#else
            VendorCtrlRqtPending = true;
#endif /* FREERTOS_ENABLE */

            /* ClearStall command to be handled in task thread. */
            isReqHandled = true;
        }

        if (bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) {
            if (wValue == CY_USB_FEATURE_DEVICE_REMOTE_WAKE) {
                /* Clear Remote Wakeup enable: ACK the request. State is tracked by stack. */
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }

            if ((wValue == CY_USB_FEATURE_U1_ENABLE) || (wValue == CY_USB_FEATURE_U2_ENABLE)) {
                /* Clear U1/U2 enable. ACK the request. Enable LPM if allowed by configuration. */
#if USB3_LPM_ENABLE
                DBG_APP_INFO("Disabling LPM\r\n");
                pUsbApp->isLpmEnabled  = false;
                pUsbApp->lpmEnableTime = 0;
                Cy_USBD_LpmDisable(pUsbdCtxt);
#endif /* USB3_LPM_ENABLE */

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == CY_USB_FEATURE_FUNC_SUSPEND)) {
            /* We only support one interface. */
            if ((wIndex & 0xFFU) == 0) {
                pUsbApp->functionWakeTimestamp = 0;
                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }
    }


    /*
     * If Request is not handled by the callback, Stall the command.
     */
    if(!isReqHandled) {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt,
                                            0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
}   /* end of function. */

void Cy_USB_AppVendorRqtHandler (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;
    cy_en_usbd_ret_code_t retStatus;
    uint16_t wLength, wValue, wIndex;
    uint8_t  bRequest, bReqType, bTarget;
    bool     isReqHandled = false;
    uint16_t loopCnt = 1000u;
    uint32_t baseAddr;
    uint16_t i;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    uint32_t epNumber;

    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bTarget  = (bReqType & CY_USB_CTRL_REQ_RECIPENT_MASK);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wLength  = pUsbdCtxt->setupReq.wLength;
    wValue   = pUsbdCtxt->setupReq.wValue;
    wIndex   = pUsbdCtxt->setupReq.wIndex;

    /* Delayed handling of ClearFeature request. */
    if ((bRequest == CY_USB_SC_CLEAR_FEATURE) && (bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) &&
            (wValue == CY_USB_FEATURE_ENDP_HALT)) {
        DBG_APP_INFO("ClearFeature: %x\r\n", wIndex);
        epDir    = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
        epNumber = (uint32_t)(wIndex & 0x7FUL);

        if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
            /* Enable clock control during EP reset to make sure all state is properly cleared. */
            Cy_USBSS_Cal_ClkStopOnEpRstEnable(pUsbdCtxt->pSsCalCtxt, true);

            /* Reset the DMA channel, endpoint and socket. */
            AbortDmaTransfer(pAppCtxt, epDir, epNumber, false);

            /* Clear the stall condition in the EP-CS register. */
            Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, epNumber, epDir, false);

            /* Re-enable the DMA channel and queue transfers as needed. */
            RestartDmaTransfer(pAppCtxt, epDir, epNumber);
        } else {
            Cy_USB_AppHandleEpReset(pAppCtxt, (uint8_t)epNumber, epDir);
        }

        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        isReqHandled = true;
    }

    if ((bRequest == DEVICE_RESET_CODE) && (bReqType == 0x40)) {
        /*
         * Device reset request:
         * 1. Get delay information.
         * 2. Initiate Status stage ACK.
         * 3. Do the reset functionality.
         */
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);

        /* Wait for wValue + 1 ms  */
        Cy_SysLib_DelayUs(((wValue+1) * 1000));
        NVIC_SystemReset();
        isReqHandled = true;
    }

    if ((bRequest == GET_DEVSPEED_CMD) && (wLength == 2)) {
        uint8_t *rspBuf_p = (uint8_t *)Ep0TestBuffer;

        rspBuf_p[0] = pUsbdCtxt->devSpeed;
        rspBuf_p[1] = pAppCtxt->desiredSpeed;
        retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, rspBuf_p, wLength);
        if (retStatus == CY_USBD_STATUS_SUCCESS)
        {
            isReqHandled = true;
        }
    }

    if ((bRequest == DEVICE_RE_ENUM_CODE) && (bReqType == 0x40)) {
        /*
         * Device re-enumeration request:
         * 1. Get delay information (wValue) and Speed
         *    information (wIndex).
         * 2. Initiate Status stage ACK.
         * 3. Set speed and then disconnect and connect device.
         */
        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        isReqHandled = true;

        if (wIndex != 0) {
            pAppCtxt->desiredSpeed = (cy_en_usb_speed_t)wIndex;
        }

        pAppCtxt->usbConnectEnabled  = false;
        pAppCtxt->reconnectTimeStamp = Cy_USBD_GetTimerTick() + (wValue * 1) * 1000;
        Cy_USB_AppSignalTask(pAppCtxt, EV_DEVSTATE_CHG);
    }

    if ((bReqType == 0x40) && (bRequest == IN_EP_CRCERR_INJ_CODE) && (wLength == 0)) {
        /*
         * Command to inject CRC32 error on egress data packet.
         * EP index in wValue
         * Packet number in wIndex
         */
        if ((wValue != 0) && (wValue < 16) && (wIndex < 16)) {
            volatile uint32_t *injcrcreg_p = (volatile uint32_t *)(0x40681144 + (wValue << 2));
            if ((*injcrcreg_p & 0x01UL) == 0) {
                *injcrcreg_p = (wIndex << 2) | 0x01UL;
            }
        }

        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        isReqHandled = true;
    }

    /* Request to get EP config information. */
    if ((bReqType == 0xC0) && (bRequest == GET_EPCONF_CMD_CODE))
    {
        uint8_t *rspBuf_p = (uint8_t *)Ep0TestBuffer;
        uint8_t idx;

        rspBuf_p[0] = pAppCtxt->numLbkPairs;
        rspBuf_p[1] = 0x00;
        rspBuf_p[2] = 0x00;
        rspBuf_p[3] = 0x00;
        for (idx = 0; idx < pAppCtxt->numLbkPairs; idx++)
        {
            rspBuf_p[((idx + 1) * 4) + 0] = idx;
            rspBuf_p[((idx + 1) * 4) + 1] = pAppCtxt->loopbackInfo[idx].LpbkEnable;
            rspBuf_p[((idx + 1) * 4) + 2] = pAppCtxt->loopbackInfo[idx].OutEndpNum;
            rspBuf_p[((idx + 1) * 4) + 3] = 0x80U | pAppCtxt->loopbackInfo[idx].InEndpNum;
        }

        if (wLength > (4 * (pAppCtxt->numLbkPairs + 1)))
        {
            wLength = (4 * (pAppCtxt->numLbkPairs + 1));
        }

        retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, wLength);
        if (retStatus == CY_USBD_STATUS_SUCCESS)
        {
            isReqHandled = true;
        }
    }

    /* Request used to test out vendor specific control request handling. */
    if ((bRequest == DATA_XFER_TEST_CODE) && (wLength != 0) && ((wValue + wLength) <= 8192U))
    {
        if ((bReqType & 0x80) != 0)
        {
            /* Evict the slow DMA cache before sending the data. */
            if (((uint32_t)Ep0TestBuffer >= 0x1C000000UL) && ((uint32_t)Ep0TestBuffer < 0x1C100000UL)) {
                Cy_HBDma_EvictReadCache(false);
            }
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, ((uint8_t *)Ep0TestBuffer) + wValue, wLength);
        }
        else
        {
            retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, ((uint8_t *)Ep0TestBuffer) + wValue, wLength);

            /* Wait until receive DMA transfer has been completed. */
            while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
                AppBlockingDelay(1);
            }

            if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
                return;
            }
        }

        if (retStatus == CY_USBD_STATUS_SUCCESS)
        {
            isReqHandled = true;
        }
        else
        {
            DBG_APP_ERR("EP0FAIL\r\n");
        }
    }

    /* Command to fetch SRAM content or MMIO registers for debug. */
    if (
            ((bRequest == REG_MEMORY_READ_CODE) || (bRequest == MEMORY_WRITE_CODE)) &&
            (wLength != 0) && ((wLength & 0x03) == 0) && (wLength <= 4096u)
       ) {
        baseAddr  = (wValue << 16U) | wIndex;
        retStatus = CY_USBD_STATUS_FAILURE;

        if ((bReqType & 0x80U) != 0) {
            if ((baseAddr >= CY_HBW_SRAM_BASE_ADDR) && (baseAddr < CY_HBW_SRAM_LAST_ADDR)) {
                Cy_HBDma_EvictReadCache(false);
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)baseAddr, wLength);
            } else {
                if (Cy_USB_IsValidMMIOAddr(baseAddr)) {
                    DBG_APP_INFO("Vendor Command 0x%x: Read from 0x%x\r\n", bRequest, baseAddr);
                    for (i = 0; i < wLength / 4; i++) {
                        Ep0TestBuffer[i] = ((uint32_t *)baseAddr)[i];
                    }

                    retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, wLength);
                }
            }

            if (retStatus == CY_USBD_STATUS_SUCCESS) {
                isReqHandled = true;
            }
        } else {

            if ((baseAddr >= CY_HBW_SRAM_BASE_ADDR) && (baseAddr < CY_HBW_SRAM_LAST_ADDR)) {
                retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)baseAddr, wLength);
                if (retStatus == CY_USBD_STATUS_SUCCESS) {
                    /* Wait until receive DMA transfer has been completed. */
                    while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
                        AppBlockingDelay(1);
                    }

                    if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                        Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
                    } else {
                        isReqHandled = true;
                    }
                }
            } else {
                if (Cy_USB_IsValidMMIOAddr(baseAddr)) {
                    DBG_APP_INFO("Vendor Command 0x%x: Write to 0x%x\r\n", bRequest, baseAddr);
                    /* Since we cannot use High BandWidth DMA to get data directly into non
                     * High BandWidth RAM regions, get the data into Ep0TestBuffer first and then copy
                     * it where it is supposed to go.
                     */
                    retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, (uint8_t *)Ep0TestBuffer, wLength);

                    if (retStatus == CY_USBD_STATUS_SUCCESS) {
                        /* Wait until receive DMA transfer has been completed. */
                        while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
                            AppBlockingDelay(1);
                        }

                        if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                            Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
                        } else {
                            isReqHandled = true;

                            for (i = 0; i < wLength / 4; i++) {
                                ((uint32_t *)baseAddr)[i] = Ep0TestBuffer[i];
                            }
                        }
                    }
                }
            }
        }
    }

    if ((bRequest == VBUS_REMOVE_SIM_CODE) && (wLength == 0))
    {
        /* Fake VBus removal for test purpose. */
        DBG_APP_INFO("VbusRemoval simulated\r\n");
        pAppCtxt->vbusPresent = false;
        Cy_USB_AppSignalTask(pAppCtxt, EV_DEVSTATE_CHG);

        Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
        isReqHandled = true;
    }

    if ((bRequest == DATA_BUF_READ_CODE) && (wLength == 0x1000U) &&
            ((bReqType & 0x80) != 0) && (wValue < pAppCtxt->numLbkPairs)) {
        retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt,
                pAppCtxt->loopbackInfo[wValue].outEpBuf[0], 0x1000U);
        if (retStatus == CY_USBD_STATUS_SUCCESS) {
            isReqHandled = true;
        }
    }

    if (
            (bRequest == UPDT_TEST_CONFIG_CODE) &&
            ((bReqType & 0x80) == 0) &&
            (wLength == sizeof(cy_stc_testdev_config_t))
       ) {
        retStatus = Cy_USB_USBD_RecvEndp0Data(pUsbdCtxt, ((uint8_t *)Ep0TestBuffer), wLength);
        if (retStatus == CY_USBD_STATUS_SUCCESS) {
            cy_stc_testdev_config_t *pNewCfg = (cy_stc_testdev_config_t *)Ep0TestBuffer;

            while ((!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) && (loopCnt--)) {
                Cy_SysLib_DelayUs(10);
            }

            if (!Cy_USBD_IsEp0ReceiveDone(pUsbdCtxt)) {
                Cy_USB_USBD_RetireRecvEndp0Data(pUsbdCtxt);
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
                return;
            }

            isReqHandled = true;

            /* If the config structure signatures are valid, copy to the special RAM area and reset. */
            if ((pNewCfg->startSig == TESTDEV_CFG_START_SIG) && (pNewCfg->endSig == TESTDEV_CFG_END_SIG)) {
                memcpy((uint8_t *)TESTDEV_CFG_STRUCT_ADDRESS, Ep0TestBuffer,
                        sizeof(cy_stc_testdev_config_t));
                NVIC_SystemReset();
            } else {
                DBG_APP_WARN("Config structure invalid\r\n");
                memset((uint8_t *)TESTDEV_CFG_STRUCT_ADDRESS, 0, sizeof(cy_stc_testdev_config_t));
            }
        } else {
            DBG_APP_ERR("Failed to get test config data: %x\r\n", retStatus);
        }
    }

    if ((bRequest == BOOT_MODE_RQT_CODE) && (wLength == 0)) {
        /* Set the boot mode request signature in RAM and reset to return to BL. */
        DBG_APP_INFO("Return to boot-loader\r\n");
        *((volatile uint32_t *)0x080003C0UL) = 0x544F4F42UL;
        *((volatile uint32_t *)0x080003C4UL) = 0x45444F4DUL;
        NVIC_SystemReset();
    }

    if (!isReqHandled) {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
    }
}

/*
 * Function: Cy_USB_AppSuspendCallback()
 * Description: This Function will be called by USBD  layer when
 *              Suspend signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSuspendCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;

#if FREERTOS_ENABLE
    /* Signal the task that device state has changed. */
    Cy_USB_AppSignalTask(pUsbApp, EV_DEVSTATE_CHG);
#endif /* FREERTOS_ENABLE */
}   /* end of function. */

/*
 * Function: Cy_USB_AppResumeCallback()
 * Description: This Function will be called by USBD  layer when
 *              Resume signal/message is detected.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppResumeCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                          cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState =  pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;

#if FREERTOS_ENABLE
    /* Signal the task that device state has changed. */
    Cy_USB_AppSignalTask(pUsbApp, EV_DEVSTATE_CHG);
#endif /* FREERTOS_ENABLE */
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppSetIntfCallback()
 * Description: This Function will be called by USBD  layer when
 *              set interface is called.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t
 * return: void
 */
void
Cy_USB_AppSetIntfCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                           cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;
    int8_t numOfEndp;
    uint8_t *pIntfDscr, *pEndpDscr;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDirection;
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("SetIntfCbk\r\n");
    pSetupReq = &(pUsbdCtxt->setupReq);

    /*
     * Get interface and alt setting info. If new setting same as previous
     * then return.
     * If new alt setting came then first Unconfigure previous settings
     * and then configure new settings.
     */
    intfNum = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

    if (altSetting == pUsbApp->prevAltSetting) {
        /* Alternate setting not changed: STALL the request to indicate no work done. */
        DBG_APP_WARN("SameAltSetting\r\n");
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, true);
        return;
    }

    /* New altSetting is different than previous one so unconfigure previous. */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pUsbApp->prevAltSetting);
    DBG_APP_TRACE("Clear previous AltSet\r\n");

    if (pIntfDscr == NULL) {
        DBG_APP_WARN("IntfDscr:NULL\r\n");
        return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp != 0x00) {
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00) {
            if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80) {
                endpDirection = CY_USB_ENDP_DIR_IN;
            } else {
                endpDirection = CY_USB_ENDP_DIR_OUT;
            }

            endpNumber = (uint32_t)((*(pEndpDscr+CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

            /* Stop/Destroy any DMA channels and disable the endpoint. */
            if (endpDirection == CY_USB_ENDP_DIR_OUT) {
                DBG_APP_INFO("SETINTF: Disable EP%d-OUT\r\n", endpNumber);
            } else {
                DBG_APP_INFO("SETINTF: Disable EP%d-IN\r\n", endpNumber);
            }

            if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                AbortDmaTransfer(pUsbApp, endpDirection, endpNumber, true);
            }

            Cy_USBD_EnableEndp(pUsbdCtxt, endpNumber, endpDirection, false);

            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
            if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
                /* Skip over SS companion descriptor. */
                pEndpDscr += 6u;
            }
        }
    }

    /* Now take care of different config with new alt setting. */
    pUsbApp->prevAltSetting = altSetting;
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, altSetting);
    if (pIntfDscr == NULL) {
        return;
    }
    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00) {
        /* TBD: This is error case need to have error log here. */
        return;
    }

    DBG_APP_INFO("SETINTF: Configuring %d endpoints\r\n", numOfEndp);
    pUsbApp->prevAltSetting = altSetting;
    pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
    while (numOfEndp != 0x00) {
        Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
        Cy_USB_AppSetupEndpDmaParams(pAppCtxt, pEndpDscr);
        numOfEndp--;
        pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        if (pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
            /* Skip over SS companion descriptor. */
            pEndpDscr += 6u;
        }
    }

    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppL1SleepCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Sleep message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppL1SleepCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    (void)pUsbApp;
    (void)pUsbdCtxt;
    (void)pMsg;

    DBG_APP_TRACE("L1SleepCbk\r\n");
    return;
}   /* end of function. */


/*
 * Function: Cy_USB_AppL1ResumeCallback()
 * Description: This Function will be called by USBD layer when
 *              L1 Resume message comes.
 * Parameter: pAppCtxt, cy_stc_usb_usbd_ctxt_t, cy_stc_usb_cal_msg_t
 * return: void
 */
void
Cy_USB_AppL1ResumeCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    (void)pUsbApp;
    (void)pUsbdCtxt;
    (void)pMsg;

    DBG_APP_TRACE("L1ResumeCbk\r\n");
    return;
}   /* end of function. */

/*
 * Function: Cy_USB_AppQueueRead()
 * Description: Function to queue read operation on an OUT endpoint.
 * Parameter: pAppCtxt, endpNumber, pBuffer, dataSize
 * return: void
 */
void
Cy_USB_AppQueueRead (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t      *dmaset_p;
#if (!USE_IP_IP_CHANNEL)
    cy_en_hbdma_mgr_status_t        hbdma_stat;
#endif /* (!USE_IP_IP_CHANNEL) */

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw0Base == NULL) || (pBuffer == NULL) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueRead: BadParam\r\n");
        return;
    }

    dmaset_p  = &(pAppCtxt->endpOutDma[endpNumber]);

    if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        if (dmaset_p->valid) {
#if (!USE_IP_IP_CHANNEL)
            /* Use HBDMA function to queue the read operation. */
            hbdma_stat = Cy_HBDma_Channel_ReceiveData(&(dmaset_p->hbDmaChannel), 0, pBuffer, dataSize, NULL);
            if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("QueueRead: HBDMA error %x\r\n", hbdma_stat);
            } else {
                DBG_APP_TRACE("RecvData: %x %x\r\n", pBuffer, dataSize);
            }
#endif /* (!USE_IP_IP_CHANNEL) */
        } else {
            DBG_APP_ERR("QueueRead: EP %d-Out is not valid\r\n", endpNumber);
        }

        return;
    }

    /* Verify that the selected endpoint is valid and the dataSize is non-zero. */
    if (dmaset_p->valid == 0)
    {
        DBG_APP_ERR("QueueRead:BadParam\r\n");
        return;
    }

    if (pAppCtxt->numLbkPairs <= 2) {
        Cy_USBHS_App_QueueReadDmaC(dmaset_p, pBuffer, dataSize);
    } else {
        Cy_USBHS_App_QueueRead(dmaset_p, pBuffer, dataSize);
    }

    if (dmaset_p->endpType != CY_USB_ENDP_TYPE_ISO) {
        /* Set transfer size for the endpoint and clear NAK status to allow data to be received. */
        if (dataSize < dmaset_p->maxPktSize) {
            /* We are trying to read out data that has already been received. Force EP NAK. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
        } else {
            /* Set the NAK bit so that the IP can see the bit transition from 1->0 after XFER_CNT is set. */
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
            Cy_USBD_UpdateXferCount(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT,
                    ((dataSize + dmaset_p->maxPktSize) & (~(dmaset_p->maxPktSize - 1))));
            Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, false);
        }
    }
} /* end of function */

/*
 * Function: Cy_USB_AppReadShortPacket
 * Description: Function to modify an ongoing DMA read operation to take care of a short packet.
 * Parameter:
 *      pAppCtxt: Application context
 *      endpNumber: Endpoint number
 *      pktSize: Size of the short packet to be read out. Can be zero in case of ZLP.
 * Return: Total size of data in the DMA buffer including data which was already read by the channel.
 */
uint16_t
Cy_USB_AppReadShortPacket(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint16_t pktSize)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;
    uint16_t dataSize = 0;

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) || (pAppCtxt->pCpuDw0Base == NULL))
    {
        DBG_APP_ERR("ReadSLP:NULL\r\n");
        return 0;
    }

    /* Verify that the selected endpoint is valid. */
    if (pAppCtxt->endpOutDma[endpNumber].valid == 0)
    {
        DBG_APP_ERR("ReadSLP:BadParam\r\n");
        return 0;
    }

    dmaset_p  = &(pAppCtxt->endpOutDma[endpNumber]);
    if (dmaset_p->endpType != CY_USB_ENDP_TYPE_ISO) {
        /* NAK the endpoint until we queue a new DMA request. */
        Cy_USB_USBD_EndpSetClearNakNrdy(pAppCtxt->pUsbdCtxt, endpNumber, CY_USB_ENDP_DIR_OUT, true);
    }

    /* The code assumes that the channel is active. */
    if (pAppCtxt->numLbkPairs <= 2) {
        dataSize = Cy_USBHS_App_ReadShortPacketDmaC(dmaset_p, pktSize);
    } else {
        dataSize = Cy_USBHS_App_ReadShortPacket(dmaset_p, pktSize);
    }

    return dataSize;
} /* end of function */

/*
 * Function: Cy_USB_AppQueueWrite()
 * Description: Function to queue write operation on an IN endpoint.
 * Parameter: pAppCtxt, endpNumber, pBuffer, dataSize
 * return: void
 */
void
Cy_USB_AppQueueWrite (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, uint8_t *pBuffer, uint16_t dataSize)
{
    cy_stc_app_endp_dma_set_t      *dmaset_p;
#if (!USE_IP_IP_CHANNEL)
    cy_en_hbdma_mgr_status_t        hbdma_stat;
#endif /* (!USE_IP_IP_CHANNEL) */

    /* Null pointer checks. */
    if ((pAppCtxt == NULL) || (pAppCtxt->pUsbdCtxt == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) || (pBuffer == NULL))
    {
        DBG_APP_ERR("QueueWrite:NULL\r\n");
        return;
    }

    dmaset_p  = &(pAppCtxt->endpInDma[endpNumber]);

    if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        if (dmaset_p->valid) {
#if (!USE_IP_IP_CHANNEL)
            /* Use HBDMA function to queue the write operation. */
            hbdma_stat = Cy_HBDma_Channel_SendData(&(dmaset_p->hbDmaChannel), 0, pBuffer, dataSize);
            if (hbdma_stat != CY_HBDMA_MGR_SUCCESS) {
                DBG_APP_ERR("QueueWrite: HBDMA error %x\r\n", hbdma_stat);
            } else {
                DBG_APP_TRACE("SendData: %x %x\r\n", pBuffer, dataSize);
            }
#endif /* (!USE_IP_IP_CHANNEL) */
        } else {
            DBG_APP_ERR("QueueWrite: EP %d-Out is not valid\r\n", endpNumber);
        }

        return;
    }

    /* Verify that the selected endpoint is valid and the dataSize is non-zero. */
    if ((dmaset_p->valid == 0) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueWrite:BadParam (%d, %d, %d)\r\n", endpNumber, dmaset_p->valid, dataSize);
        return;
    }

    if (
            (pAppCtxt->numLbkPairs <= 2)
       ) {
        Cy_USBHS_App_QueueWriteDmaC(dmaset_p, pBuffer, dataSize);
    } else {
        Cy_USBHS_App_QueueWrite(dmaset_p, pBuffer, dataSize);
    }
} /* end of function */


/*
 * Function: Cy_USB_AppTerminateDma()
 * Description: Function to cancel any ongoing DMA operations on an endpoint.
 * Parameter: pAppCtxt, endpNumber, endpDirection
 * return: void
 */
void
Cy_USB_AppTerminateDma (cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    cy_stc_app_endp_dma_set_t *dmaset_p;

    /* Null pointer checks. */
    if (
            (pAppCtxt == NULL) ||
            (pAppCtxt->pCpuDw0Base == NULL) ||
            (pAppCtxt->pCpuDw1Base == NULL) ||
            (pAppCtxt->pCpuDmacBase == NULL)
       )
    {
        DBG_APP_ERR("TerminateDma:NULL\r\n");
        return;
    }

    if (pAppCtxt->pUsbdCtxt->devSpeed > CY_USBD_USB_DEV_HS) {
        if (endpDirection == CY_USB_ENDP_DIR_OUT) {
            dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);
            if (dmaset_p->valid) {
                Cy_HBDma_Channel_Disable(&(dmaset_p->hbDmaChannel));
            }
        } else {
            dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);
            if (dmaset_p->valid) {
                Cy_HBDma_Channel_Disable(&(dmaset_p->hbDmaChannel));
            }
        }
    } else {
        /* If the DMA channel is already enabled, disable it. */
        if (endpDirection == CY_USB_ENDP_DIR_OUT) {
            dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);
            if (pAppCtxt->numLbkPairs <= 2) {
                Cy_USBHS_App_ResetEpDmaC(dmaset_p);
            } else {
                Cy_USBHS_App_ResetEpDma(dmaset_p);
            }
        } else {
            dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);
            if (pAppCtxt->numLbkPairs <= 2) {
                Cy_USBHS_App_ResetEpDmaC(dmaset_p);
            } else {
                Cy_USBHS_App_ResetEpDma(dmaset_p);
            }
        }
    }

    return;
}   /* end of function */


/*
 * Function: Cy_USB_AppInitDmaIntr()
 * Description: Function to register an ISR for the DMA channel associated
 *              with an endpoint.
 * Parameters:
 *      pAppCtxt: Application context pointer.
 *      endpNumber: Endpoint number
 *      endpDirection: Endpoint direction.
 *      userIsr: ISR function pointer. Can be NULL if interrupt is to be disabled.
 * return: void
 */
void
Cy_USB_AppInitDmaIntr (cy_stc_usb_app_ctxt_t *pAppCtxt, uint32_t endpNumber,
        cy_en_usb_endp_dir_t endpDirection, cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    cy_stc_app_endp_dma_set_t *dmaset_p;

    if ((pAppCtxt != NULL) && (endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {

#if (CY_CPU_CORTEX_M4)
        intrCfg.intrPriority = 5;
#else
        intrCfg.intrPriority = 3;
#endif /* (CY_CPU_CORTEX_M4) */

        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            dmaset_p = &(pAppCtxt->endpInDma[endpNumber]);

#if (CY_CPU_CORTEX_M4)
            if (pAppCtxt->numLbkPairs <= 2)
                intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dmac_0_IRQn + dmaset_p->channel);
            else
                intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + dmaset_p->channel);
#else
            /* DW1 channels are used for IN endpoints. */
            intrCfg.intrSrc = NvicMux6_IRQn;
            if (pAppCtxt->numLbkPairs <= 2)
                intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dmac_0_IRQn + dmaset_p->channel);
            else
                intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + dmaset_p->channel);
#endif /* (CY_CPU_CORTEX_M4) */
        } else {
            dmaset_p = &(pAppCtxt->endpOutDma[endpNumber]);

#if (CY_CPU_CORTEX_M4)
            if (pAppCtxt->numLbkPairs <= 2)
                intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dmac_0_IRQn + dmaset_p->channel);
            else
                intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + dmaset_p->channel);
#else
            /* DW0 channels are used for OUT endpoints. */
            intrCfg.intrSrc = NvicMux5_IRQn;
            if (pAppCtxt->numLbkPairs <= 2)
                intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dmac_0_IRQn + dmaset_p->channel);
            else
                intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + dmaset_p->channel);
#endif /* (CY_CPU_CORTEX_M4) */
        }

        if (userIsr != NULL)  {
            /* If an ISR is provided, register it and enable the interrupt. */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        } else {
            /* ISR is NULL. Disable the interrupt. */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
} /* end of function. */

/*
 * Function: Cy_USB_AppClearDmaInterrupt()
 * Description: Function to clear the pending DMA interrupt associated with an endpoint.
 * Parameters:
 *      pAppCtxt: Pointer to USB application context structure.
 *      endpNumber: Endpoint number
 *      endpDirection: Endpoint direction.
 * return: void
 */
void
Cy_USB_AppClearDmaInterrupt(cy_stc_usb_app_ctxt_t *pAppCtxt,
        uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection)
{
    if ((pAppCtxt != NULL) && (endpNumber > 0) &&
        (endpNumber < CY_USB_MAX_ENDP_NUMBER)) {
        if (endpDirection == CY_USB_ENDP_DIR_IN) {
            if (pAppCtxt->numLbkPairs <= 2) {
                Cy_USBHS_App_ClearDmaCInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
            } else {
                Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpInDma[endpNumber]));
            }
        } else  {
            if (pAppCtxt->numLbkPairs <= 2) {
                Cy_USBHS_App_ClearDmaCInterrupt(&(pAppCtxt->endpOutDma[endpNumber]));
            } else {
                Cy_USBHS_App_ClearDmaInterrupt(&(pAppCtxt->endpOutDma[endpNumber]));
            }
        }
    }
}   /* end of function. */


/*
 * Function: Cy_USB_AppFindValidInEndpNumber()
 * Description: Finds IN endpoint which is configured.
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: uint8_t
 */
uint8_t
Cy_USB_AppFindValidInEndpNumber (cy_stc_usb_app_ctxt_t *pUsbApp)
{
    uint8_t endpNum = 0x00;
    uint8_t index;
    for (index = 0x01; index < CY_USB_MAX_ENDP_NUMBER; index++) {
        if  (pUsbApp->endpInDma[index].valid) {
            endpNum = index;
            break;
        }
    }
    return (endpNum);
}   /* end of function */


/*
 * Function: Cy_USB_AppFindValidOutEndpNumber()
 * Description: Finds OUT endpoint which is configured.
 * Parameter: cy_stc_usb_app_ctxt_t
 * return: uint8_t
 */
uint8_t
Cy_USB_AppFindValidOutEndpNumber (cy_stc_usb_app_ctxt_t *pUsbApp)
{
    uint8_t endpNum = 0x00;
    uint8_t index;
    for (index = 0x01; index < CY_USB_MAX_ENDP_NUMBER; index++) {
        if  (pUsbApp->endpOutDma[index].valid) {
            endpNum = index;
            break;
        }
    }
    return (endpNum);
}   /* end of function */


/*
 * Function: Cy_USB_AppGetMaxPktSize()
 * Description: Finds maxPktSize.
 * Parameter: cy_stc_usb_app_ctxt_t, endpNum, dir
 * return: uint8_t
 */
uint32_t
Cy_USB_AppGetMaxPktSize (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t endpNum,
                                                    cy_en_usb_endp_dir_t dir)
{

    uint32_t maxPktSize = 0x00;
    if (dir == CY_USB_ENDP_DIR_IN) {
        maxPktSize =   pUsbApp->endpInDma[endpNum].maxPktSize;
    } else {
        maxPktSize =   pUsbApp->endpOutDma[endpNum].maxPktSize;
    }
    return (maxPktSize);
}   /* end of function */

/*[]*/
