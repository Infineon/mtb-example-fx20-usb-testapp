/***************************************************************************//**
* \file cy_usbhs_dmac_wrapper.c
* \version 1.0
*
* Provides wrapper functions that enable usage of DMAC channels to read/write
* data from/to the USBHS IP endpoint memory.
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

#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usbhs_dmac_wrapper.h"
#include "cy_debug.h"

/*
 * Function: Cy_USBHS_App_EnableEpDmaCSet()
 * Description: Configure DMA resources related to an USBHS endpoint.
 * Parameters:
 *      pEpDmaSet   : Pointer to Endpoint DMA context data structure.
 *      pDmacStruct : Pointer to DMAC register structure.
 *      channelNum  : DMAC Channel number.
 *      epNumber    : Endpoint number.
 *      epDir       : Endpoint direction.
 *      maxPktSize  : Maximum packet size for the endpoint.
 * Return: true if endpoint DMA init is complete, false otherwise.
 */
bool
Cy_USBHS_App_EnableEpDmaCSet (
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        DMAC_Type                 *pDmacStruct,
        uint8_t                    channelNum,
        uint8_t                    epNumber,
        cy_en_usb_endp_dir_t       epDir,
        uint16_t                   maxPktSize)
{
    bool stat = false;

    /* Check whether all parameters are valid. */
    if ((pEpDmaSet != NULL) && (pDmacStruct != NULL) && (epNumber < CY_USB_MAX_ENDP_NUMBER) && (maxPktSize != 0))
    {
        pEpDmaSet->pDwStruct    = (void *)pDmacStruct;
        pEpDmaSet->channel      = channelNum;
        pEpDmaSet->epNumber     = epNumber;
        pEpDmaSet->epDir        = epDir;
        pEpDmaSet->maxPktSize   = maxPktSize;
        pEpDmaSet->valid        = true;
        pEpDmaSet->firstRqtDone = false;
        pEpDmaSet->pCurDataBuffer = NULL;
        pEpDmaSet->curDataSize  = 0;

        if (epDir == CY_USB_ENDP_DIR_IN)
        {
            /* Make the trigger connections from USB endpoint to DMAC. */
            Cy_TrigMux_Connect(TRIG_IN_MUX_5_USBHSDEV_TR_OUT16 + epNumber,
                    TRIG_OUT_MUX_5_MDMA_TR_IN0 + channelNum,
                    false, TRIGGER_TYPE_LEVEL);
            /* Make the trigger connections from DMAC to USB endpoint. */
            Cy_TrigMux_Connect(TRIG_IN_MUX_8_MDMA_TR_OUT0 + channelNum,
                    TRIG_OUT_MUX_8_USBHSDEV_TR_IN16 + epNumber,
                    false, TRIGGER_TYPE_LEVEL);
        }
        else
        {
            /* Make the trigger connections from USB endpoint to DMAC. */
            Cy_TrigMux_Connect(TRIG_IN_MUX_5_USBHSDEV_TR_OUT0 + epNumber,
                    TRIG_OUT_MUX_5_MDMA_TR_IN0 + channelNum,
                    false, TRIGGER_TYPE_LEVEL);
            /* Make the trigger connections from DMAC to USB endpoint. */
            Cy_TrigMux_Connect(TRIG_IN_MUX_8_MDMA_TR_OUT0 + channelNum,
                    TRIG_OUT_MUX_8_USBHSDEV_TR_IN0 + epNumber,
                    false, TRIGGER_TYPE_LEVEL);
        }

        stat = true;
    }
    else
    {
        DBG_APP_ERR("EnableEpDma: Invalid parameters\r\n");
    }

    return stat;
}

/*
 * Function: Cy_USBHS_App_DisableEpDmaCSet()
 * Description: De-init DMA resources related to an USBHS endpoint.
 * Parameters:
 *      pEpDmaSet : Pointer to Endpoint DMA context data structure.
 * Return: void
 */
void
Cy_USBHS_App_DisableEpDmaCSet (
        cy_stc_app_endp_dma_set_t *pEpDmaSet)
{
    DMAC_Type *pDmacStruct;

    if (pEpDmaSet != NULL)
    {
        /* If the endpoint was valid, make sure that triggers are disconnected. */
        if (pEpDmaSet->valid)
        {
            pDmacStruct = (DMAC_Type *)(pEpDmaSet->pDwStruct);

            /* Make sure the DMAC channel is disabled. */
            Cy_DMAC_Channel_Disable(pDmacStruct, pEpDmaSet->channel);
            Cy_DMAC_Channel_DeInit(pDmacStruct, pEpDmaSet->channel);

            /* Disconnect the triggers between DMAC and USBHS Endpoint. */
            if (pEpDmaSet->epDir == CY_USB_ENDP_DIR_IN)
            {
                Cy_TrigMux_Connect((TRIG_IN_MUX_5_PDMA0_TR_OUT0 & 0xFFFFFF00UL),
                        TRIG_OUT_MUX_5_MDMA_TR_IN0 + pEpDmaSet->channel,
                        false, TRIGGER_TYPE_LEVEL);
                Cy_TrigMux_Connect((TRIG_IN_MUX_8_MDMA_TR_OUT0 & 0xFFFFFF00UL),
                        TRIG_OUT_MUX_8_USBHSDEV_TR_IN16 + pEpDmaSet->epNumber,
                        false, TRIGGER_TYPE_LEVEL);
            }
            else
            {
                Cy_TrigMux_Connect((TRIG_IN_MUX_5_PDMA0_TR_OUT0 & 0xFFFFFF00UL),
                        TRIG_OUT_MUX_5_MDMA_TR_IN0 + pEpDmaSet->channel,
                        false, TRIGGER_TYPE_LEVEL);
                Cy_TrigMux_Connect((TRIG_IN_MUX_8_MDMA_TR_OUT0 & 0xFFFFFF00UL),
                        TRIG_OUT_MUX_8_USBHSDEV_TR_IN0 + pEpDmaSet->epNumber,
                        false, TRIGGER_TYPE_LEVEL);
            }
        }

        /* Clear important structure fields. */
        pEpDmaSet->pDwStruct    = NULL;
        pEpDmaSet->valid        = false;
        pEpDmaSet->firstRqtDone = false;
        pEpDmaSet->channel      = 0;
        pEpDmaSet->epNumber     = 0;
        pEpDmaSet->pCurDataBuffer = NULL;
        pEpDmaSet->curDataSize  = 0;
    }
}

/*
 * Function: Cy_USBHS_App_ResetEpDmaC()
 * Description: Reset the DMA resources corresponding to an endpoint.
 * Parameters:
 *      pEpDmaSet : Pointer to Endpoint DMA context data structure.
 * Return: void
 */
void
Cy_USBHS_App_ResetEpDmaC (
        cy_stc_app_endp_dma_set_t *pEpDmaSet)
{
    if ((pEpDmaSet != NULL) && (pEpDmaSet->valid))
    {
        /* Make sure the DMAC channel is disabled. */
        Cy_DMAC_Channel_Disable((DMAC_Type *)pEpDmaSet->pDwStruct, pEpDmaSet->channel);

        /* Clear the first request done flag. */
        pEpDmaSet->firstRqtDone = false;
    }
}

/*
 * Function: Cy_USBHS_App_QueueReadDmaC()
 * Description: Function to queue read operation on an OUT endpoint.
 * Parameters:
 *     pEpDmaSet: Pointer to Endpoint DMA context data structure.
 *     pBuffer  : Pointer to buffer into which data should be read.
 *     dataSize : Size of data to be read in bytes.
 * Return: true if read is queued, false if not.
 */
bool
Cy_USBHS_App_QueueReadDmaC (
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint8_t                   *pBuffer,
        uint16_t                   dataSize)
{
    cy_stc_dmac_descriptor_t        *pFirstDscr;
    cy_stc_dmac_descriptor_config_t  dmaDscrConfig;
    cy_stc_dmac_channel_config_t     dmaChannelConfig;
    uint16_t   residue;
    uint16_t   nFullPkts;
    uint32_t  *epmAddr;
    DMAC_Type *pDmacStruct;

    /* Null pointer checks. */
    if ((pEpDmaSet == NULL) || (pEpDmaSet->pDwStruct == NULL) || (pBuffer == NULL))
    {
        DBG_APP_ERR("QueueRead:NULL\r\n");
        return false;
    }

    /* Verify that the selected endpoint is valid and the dataSize is non-zero. */
    if ((pEpDmaSet->valid == 0) || (pEpDmaSet->epDir != CY_USB_ENDP_DIR_OUT) ||
            (pEpDmaSet->maxPktSize == 0) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueRead:BadParam\r\n");
        return false;
    }

    /* Maximum number of packets per descriptor is 256. */
    if (dataSize >= (pEpDmaSet->maxPktSize << 8U)) {
        nFullPkts = 256U;
    } else {
        nFullPkts = (dataSize / pEpDmaSet->maxPktSize);
    }

    residue   = (dataSize % pEpDmaSet->maxPktSize);
    epmAddr   = Cy_USBHS_CalculateEpmAddr(pEpDmaSet->epNumber, CY_USB_ENDP_DIR_OUT);

    pFirstDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1);

    /* If the DMA channel is already enabled, disable it. */
    pDmacStruct = (DMAC_Type *)pEpDmaSet->pDwStruct;
    Cy_DMAC_Channel_Disable(pDmacStruct, pEpDmaSet->channel);

    /* If 1 or more full packets are to be read out, set up the 2-D DMA descriptor. */
    if (nFullPkts != 0)
    {
        pFirstDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 0);

        dmaDscrConfig.dataPrefetch         = false;
        dmaDscrConfig.retrigger            = CY_DMAC_WAIT_FOR_REACT;
        dmaDscrConfig.interruptType        = CY_DMAC_DESCR_CHAIN;
        dmaDscrConfig.triggerInType        = CY_DMAC_X_LOOP;
        dmaDscrConfig.triggerOutType       = CY_DMAC_X_LOOP;
        dmaDscrConfig.channelState         = (residue != 0) ? CY_DMAC_CHANNEL_ENABLED : CY_DMAC_CHANNEL_DISABLED;
        dmaDscrConfig.dataSize             = CY_DMAC_WORD;
        dmaDscrConfig.srcTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
        dmaDscrConfig.dstTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
        dmaDscrConfig.descriptorType       = CY_DMAC_2D_TRANSFER;
        dmaDscrConfig.srcAddress           = (void *)epmAddr;
        dmaDscrConfig.srcXincrement        = 1u;
        dmaDscrConfig.srcYincrement        = 0;
        dmaDscrConfig.dstAddress           = (void *)pBuffer;
        dmaDscrConfig.dstXincrement        = 1u;
        dmaDscrConfig.dstYincrement        = (pEpDmaSet->maxPktSize >> 2u);
        dmaDscrConfig.xCount               = (pEpDmaSet->maxPktSize >> 2u);
        dmaDscrConfig.yCount               = nFullPkts;
        if (residue != 0) {
            if (residue >= 0x04U) {
                dmaDscrConfig.nextDescriptor = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1);
            } else {
                dmaDscrConfig.nextDescriptor = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2);
            }
        } else {
            dmaDscrConfig.nextDescriptor   = NULL;
        }

        Cy_DMAC_Descriptor_Init(pFirstDscr, &dmaDscrConfig);
    }
    else
    {
        /* Zero out descriptor[0] since it is not being used. */
        memset((uint8_t *)(&pEpDmaSet->dmaXferDscr[0]), 0, sizeof(cy_stc_usbhs_dma_desc_t));
    }

    /* If there is a short packet to be read at the end, set up descriptors as required. */
    if (residue != 0)
    {
        if (residue >= 0x04U)
        {
            dmaDscrConfig.dataPrefetch         = false;
            dmaDscrConfig.retrigger            = CY_DMAC_WAIT_FOR_REACT;
            dmaDscrConfig.interruptType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.dataSize             = CY_DMAC_WORD;
            dmaDscrConfig.srcTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMAC_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = epmAddr;
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)(pBuffer + nFullPkts * pEpDmaSet->maxPktSize);
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue >> 2u);
            dmaDscrConfig.yCount               = 0;
            if ((residue & 0x03U) != 0) {
                dmaDscrConfig.channelState     = CY_DMAC_CHANNEL_ENABLED;
                dmaDscrConfig.nextDescriptor   = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2);
            } else {
                dmaDscrConfig.channelState     = CY_DMAC_CHANNEL_DISABLED;
                dmaDscrConfig.nextDescriptor   = 0;
            }
            Cy_DMAC_Descriptor_Init(Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1), &dmaDscrConfig);
        }
        else
        {
            if (nFullPkts == 0)
            {
                pFirstDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2);
            }
        }

        if ((residue & 0x03U) != 0)
        {
            dmaDscrConfig.dataPrefetch         = false;
            dmaDscrConfig.retrigger            = CY_DMAC_RETRIG_IM;
            dmaDscrConfig.interruptType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.channelState         = CY_DMAC_CHANNEL_DISABLED;
            dmaDscrConfig.dataSize             = CY_DMAC_BYTE;
            dmaDscrConfig.srcTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMAC_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = (void *)(epmAddr + (residue >> 2u));
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)(pBuffer + (dataSize & 0xFFFCU));
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue & 0x03U);
            dmaDscrConfig.yCount               = 0;
            dmaDscrConfig.nextDescriptor       = NULL;
            Cy_DMAC_Descriptor_Init(Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2), &dmaDscrConfig);
        }
    }

    dmaChannelConfig.descriptor  = pFirstDscr;
    dmaChannelConfig.priority    = 3;
    dmaChannelConfig.enable      = false;
    dmaChannelConfig.bufferable  = false;
    Cy_DMAC_Channel_Init(pDmacStruct, pEpDmaSet->channel, &dmaChannelConfig);

    Cy_DMAC_Channel_SetInterruptMask(pDmacStruct, pEpDmaSet->channel, CY_DMAC_INTR_MASK);
    Cy_DMAC_Channel_Enable(pDmacStruct, pEpDmaSet->channel);

    pEpDmaSet->firstRqtDone = true;
    return true;
}

/*
 * Function: Cy_USBHS_App_ReadShortPacketDmaC
 * Description: Function to modify an ongoing DMA read operation to take care of a short packet.
 * Parameter:
 *      pEpDmaSet : Pointer to Endpoint DMA context data structure.
 *      pktSize   : Size of the short packet to be read out. Can be zero in case of ZLP.
 * Return: Total size of data in the DMA buffer including data which was already read by the channel.
 */
uint16_t
Cy_USBHS_App_ReadShortPacketDmaC (
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint16_t                   pktSize)
{
    cy_stc_dmac_descriptor_t   *activeDscr;
    cy_stc_dmac_descriptor_t   *prevDscr;
    uint16_t dataSize = 0;
    uint8_t *pDataBuf = NULL;
    DMAC_Type *pDmacStruct;

    /* Null pointer checks. */
    if ((pEpDmaSet == NULL) || (pEpDmaSet->pDwStruct == NULL))
    {
        DBG_APP_ERR("ReadSLP:NULL\r\n");
        return 0;
    }

    /* Verify that the selected endpoint is valid. */
    if ((pEpDmaSet->valid == 0) || (pEpDmaSet->epDir != CY_USB_ENDP_DIR_OUT))
    {
        DBG_APP_ERR("ReadSLP:BadParam\r\n");
        return 0;
    }

    pDmacStruct = (DMAC_Type *)pEpDmaSet->pDwStruct;

    /* Get the current descriptor and check whether it is the 2-D descriptor. */
    activeDscr = Cy_DMAC_Channel_GetCurrentDescriptor(pDmacStruct, pEpDmaSet->channel);
    if (
            (activeDscr != Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 0)) &&
            (activeDscr != Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1))
       )
    {
        DBG_APP_ERR("ReadSLP:BadState\r\n");
        return 0;
    }

    if (activeDscr == Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 0))
    {
        /*
         * Channel is still working on Descriptor #0. Calculate the size of data received so far by
         * checking the current Y index of the channel. Then disable and re-enable the DMA channel with
         * the appropriate buffer address and short packet size.
         */
        dataSize = Cy_DMAC_Channel_GetCurrentYloopIndex(pDmacStruct, pEpDmaSet->channel) * pEpDmaSet->maxPktSize;
        pDataBuf = (uint8_t *)Cy_DMAC_Descriptor_GetDstAddress(activeDscr);
        pDataBuf += dataSize;
        dataSize += pktSize;
    }
    else
    {
        prevDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 0);

        /*
         * Descriptor #0 is complete. We can take all the data corresponding to Descriptor #0 as received.
         * Read is to be resumed from the destination address programmed in Descriptor #1.
         */
        if (prevDscr->ctl != 0) {
            dataSize = Cy_DMAC_Descriptor_GetYloopDataCount(prevDscr) * pEpDmaSet->maxPktSize;
        } else {
            /* Descriptor #0 was not in use. */
            dataSize = 0;
        }

        pDataBuf = (uint8_t *)Cy_DMAC_Descriptor_GetDstAddress(activeDscr);
        dataSize += pktSize;
    }

    if (pktSize != 0)
    {
        Cy_USBHS_App_QueueReadDmaC(pEpDmaSet, pDataBuf, pktSize);
    }
    else
    {
        /* Disable the DMA channel. */
        Cy_DMAC_Channel_Disable(pDmacStruct, pEpDmaSet->channel);
    }

    return dataSize;
}

/*
 * Function: Cy_USBHS_App_QueueWriteDmaC()
 * Description: Function to queue write operation on an IN endpoint.
 * Parameters:
 *     pEpDmaSet: Pointer to Endpoint DMA context data structure.
 *     pBuffer  : Pointer to buffer with the data to be written.
 *     dataSize : Size of data to be written in bytes.
 * Return: true if write is queued, false if not.
 */
bool
Cy_USBHS_App_QueueWriteDmaC (
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint8_t                   *pBuffer,
        uint16_t                   dataSize)
{
    cy_stc_dmac_descriptor_t        *pFirstDscr;
    cy_stc_dmac_descriptor_config_t  dmaDscrConfig;
    cy_stc_dmac_channel_config_t     dmaChannelConfig;
    uint32_t   *epmAddr;
    uint16_t    residue;
    uint16_t    nFullPkts;
    DMAC_Type  *pDmacStruct;

    /* Null pointer checks. */
    if ((pEpDmaSet == NULL) || (pEpDmaSet->pDwStruct == NULL) || (pBuffer == NULL))
    {
        DBG_APP_ERR("QueueWrite:NULL\r\n");
        return false;
    }

    /* Verify that the selected endpoint is valid and the dataSize is non-zero. */
    if ((pEpDmaSet->valid == 0) || (pEpDmaSet->epDir != CY_USB_ENDP_DIR_IN) ||
            (pEpDmaSet->maxPktSize == 0) || (dataSize == 0))
    {
        DBG_APP_ERR("QueueWrite:BadParam\r\n");
        return false;
    }

    /* Store the current transfer details. */
    pEpDmaSet->pCurDataBuffer = pBuffer;
    pEpDmaSet->curDataSize    = dataSize;

    /* Maximum number of packets per descriptor is 256. */
    if (dataSize >= (pEpDmaSet->maxPktSize << 8U)) {
        nFullPkts = 256U;
    } else {
        nFullPkts = (dataSize / pEpDmaSet->maxPktSize);
    }
    residue   = (dataSize % pEpDmaSet->maxPktSize);
    epmAddr   = Cy_USBHS_CalculateEpmAddr(pEpDmaSet->epNumber, CY_USB_ENDP_DIR_IN);

    pFirstDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1);

    /* If the DMA channel is already enabled, disable it. */
    pDmacStruct = (DMAC_Type *)pEpDmaSet->pDwStruct;
    Cy_DMAC_Channel_Disable(pDmacStruct, pEpDmaSet->channel);

    /* If 1 or more full packets are to be written out, set up the 2-D DMA descriptor. */
    if (nFullPkts != 0)
    {
        pFirstDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 0);

        dmaDscrConfig.dataPrefetch         = false;
        dmaDscrConfig.retrigger            = CY_DMAC_WAIT_FOR_REACT;
        dmaDscrConfig.interruptType        = CY_DMAC_DESCR_CHAIN;
        dmaDscrConfig.triggerInType        = CY_DMAC_X_LOOP;
        dmaDscrConfig.triggerOutType       = CY_DMAC_X_LOOP;
        dmaDscrConfig.channelState         = (residue != 0) ? CY_DMAC_CHANNEL_ENABLED : CY_DMAC_CHANNEL_DISABLED;
        dmaDscrConfig.dataSize             = CY_DMAC_WORD;
        dmaDscrConfig.srcTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
        dmaDscrConfig.dstTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
        dmaDscrConfig.descriptorType       = CY_DMAC_2D_TRANSFER;
        dmaDscrConfig.srcAddress           = (void *)pBuffer;
        dmaDscrConfig.srcXincrement        = 1u;
        dmaDscrConfig.srcYincrement        = (pEpDmaSet->maxPktSize >> 2u);
        dmaDscrConfig.dstAddress           = (void *)epmAddr;
        dmaDscrConfig.dstXincrement        = 1u;
        dmaDscrConfig.dstYincrement        = 0;
        dmaDscrConfig.xCount               = (pEpDmaSet->maxPktSize >> 2u);
        dmaDscrConfig.yCount               = nFullPkts;
        if (residue != 0) {
            if (residue >= 4U) {
                dmaDscrConfig.nextDescriptor = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1);
            } else {
                dmaDscrConfig.nextDescriptor = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2);
            }
        } else {
            dmaDscrConfig.nextDescriptor   = NULL;
        }

        Cy_DMAC_Descriptor_Init(pFirstDscr, &dmaDscrConfig);
    }

    /* If there is a short packet to be read at the end, set up descriptors as required. */
    if (residue != 0)
    {
        if (residue >= 0x04U)
        {
            dmaDscrConfig.dataPrefetch         = false;
            dmaDscrConfig.retrigger            = CY_DMAC_WAIT_FOR_REACT;
            dmaDscrConfig.interruptType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.dataSize             = CY_DMAC_WORD;
            dmaDscrConfig.srcTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMAC_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = (void *)(pBuffer + nFullPkts * pEpDmaSet->maxPktSize);
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)epmAddr;
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue >> 2u);
            dmaDscrConfig.yCount               = 0;
            if ((residue & 0x03U) != 0) {
                dmaDscrConfig.channelState     = CY_DMAC_CHANNEL_ENABLED;
                dmaDscrConfig.nextDescriptor   = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2);
            } else {
                dmaDscrConfig.channelState     = CY_DMAC_CHANNEL_DISABLED;
                dmaDscrConfig.nextDescriptor   = NULL;
            }
            Cy_DMAC_Descriptor_Init(Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 1), &dmaDscrConfig);
        }
        else
        {
            if (nFullPkts == 0)
            {
                pFirstDscr = Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2);
            }
        }

        if ((residue & 0x03U) != 0)
        {
            dmaDscrConfig.dataPrefetch         = false;
            dmaDscrConfig.retrigger            = CY_DMAC_RETRIG_IM;
            dmaDscrConfig.interruptType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerInType        = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.triggerOutType       = CY_DMAC_DESCR_CHAIN;
            dmaDscrConfig.channelState         = CY_DMAC_CHANNEL_DISABLED;
            dmaDscrConfig.dataSize             = CY_DMAC_BYTE;
            dmaDscrConfig.srcTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.dstTransferSize      = CY_DMAC_TRANSFER_SIZE_DATA;
            dmaDscrConfig.descriptorType       = CY_DMAC_1D_TRANSFER;
            dmaDscrConfig.srcAddress           = (void *)(pBuffer + (dataSize & 0xFFFCU));
            dmaDscrConfig.srcXincrement        = 1u;
            dmaDscrConfig.srcYincrement        = 0;
            dmaDscrConfig.dstAddress           = (void *)(epmAddr + (residue >> 2u));
            dmaDscrConfig.dstXincrement        = 1u;
            dmaDscrConfig.dstYincrement        = 0;
            dmaDscrConfig.xCount               = (residue & 0x03U);
            dmaDscrConfig.yCount               = 0;
            dmaDscrConfig.nextDescriptor       = NULL;
            Cy_DMAC_Descriptor_Init(Cy_UsbHS_App_GetDmaCDesc(pEpDmaSet, 2), &dmaDscrConfig);
        }
    }

    dmaChannelConfig.descriptor  = pFirstDscr;
    dmaChannelConfig.priority    = 3;
    dmaChannelConfig.enable      = false;
    dmaChannelConfig.bufferable  = false;
    Cy_DMAC_Channel_Init(pDmacStruct, pEpDmaSet->channel, &dmaChannelConfig);
    Cy_DMAC_Channel_SetInterruptMask(pDmacStruct, pEpDmaSet->channel, CY_DMAC_INTR_MASK);

    /* Make sure the HBW SRAM read cache is flushed when starting a new DMA operation. */
    if (((uint32_t)pBuffer >= CY_HBW_SRAM_BASE_ADDR) && ((uint32_t)pBuffer < CY_HBW_SRAM_LAST_ADDR)) {
        Cy_HBDma_EvictReadCache(false);
    }

    Cy_DMAC_Channel_Enable(pDmacStruct, pEpDmaSet->channel);

    /* For the first transfer on an IN endpoint, we need to assert the trigger manually. */
    if (pEpDmaSet->firstRqtDone == false)
    {
        Cy_TrigMux_SwTrigger(TRIG_IN_MUX_5_USBHSDEV_TR_OUT16 + pEpDmaSet->epNumber, CY_TRIGGER_TWO_CYCLES);
    }

    pEpDmaSet->firstRqtDone = true;
    return true;
}

/*
 * Function: Cy_USBHS_App_ClearDmaCInterrupt()
 * Description: Clear pending DataWire channel interrupts for a given
 * endpoint.
 * Parameters:
 *     pEpDmaSet: Pointer to Endpoint DMA context data structure.
 * Return: void
 */
void
Cy_USBHS_App_ClearDmaCInterrupt (
        cy_stc_app_endp_dma_set_t *pEpDmaSet)
{
    if ((pEpDmaSet != NULL) && (pEpDmaSet->pDwStruct != NULL))
    {
        Cy_DMAC_Channel_ClearInterrupt((DMAC_Type *)pEpDmaSet->pDwStruct, pEpDmaSet->channel, CY_DMAC_INTR_MASK);
    }
}

/*[]*/

