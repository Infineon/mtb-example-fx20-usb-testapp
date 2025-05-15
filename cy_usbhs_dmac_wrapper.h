/***************************************************************************//**
* \file cy_usbhs_dmac_wrapper.h
* \version 1.0
*
* Defines the interfaces provided to enable data transfers from/to the USBHS
* IP endpoint memories using DMAC channels.
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


#ifndef _CY_USBHS_DMAC_WRAPPER_H_
#define _CY_USBHS_DMAC_WRAPPER_H_

#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

/******************* Function Prototypes *********************/

/* Configure the DMA resources associated with a USBHS endpoint. */
bool
Cy_USBHS_App_EnableEpDmaCSet(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        DMAC_Type                 *pDmacStruct,
        uint8_t                    channelNum,
        uint8_t                    epNumber,
        cy_en_usb_endp_dir_t       epDir,
        uint16_t                   maxPktSize);


/* De-init DMA resources related to an USBHS endpoint. */
void
Cy_USBHS_App_DisableEpDmaCSet(
        cy_stc_app_endp_dma_set_t *pEpDmaSet);

/* Reset the DMA resources corresponding to an endpoint. */
void
Cy_USBHS_App_ResetEpDmaC(
        cy_stc_app_endp_dma_set_t *pEpDmaSet);

/* Function to queue read operation on an OUT endpoint. */
bool
Cy_USBHS_App_QueueReadDmaC(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint8_t                   *pBuffer,
        uint16_t                   dataSize);

/* Function to modify an ongoing DMA read operation to take care of a short packet. */
uint16_t
Cy_USBHS_App_ReadShortPacketDmaC(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint16_t                   pktSize);

/* Function to queue write operation on an IN endpoint. */
bool
Cy_USBHS_App_QueueWriteDmaC(
        cy_stc_app_endp_dma_set_t *pEpDmaSet,
        uint8_t                   *pBuffer,
        uint16_t                   dataSize);

/* Disable any pending DataWire channel interrupts for an endpoint. */
void
Cy_USBHS_App_ClearDmaCInterrupt(
        cy_stc_app_endp_dma_set_t *pEpDmaSet);

#if defined(__cplusplus)
}
#endif

#endif /* _CY_USBHS_DMAC_WRAPPER_H_ */

/* End of File */

