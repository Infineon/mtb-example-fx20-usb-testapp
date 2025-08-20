/***************************************************************************//**
* \file main.c
* \version 1.0
*
* Main source file of the FX20 USB test application.
*
*******************************************************************************
* \copyright
* (c) (2023-2025), Cypress Semiconductor Corporation (an Infineon company) or
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
#include "timers.h"
#include "cy_pdl.h"
#include <string.h>
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"
#include "cy_usbd_version.h"
#include "cy_fault_handlers.h"
#include "cy_debug.h"
#include "cy_fx_common.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_hbdma_version.h"
#include "app_version.h"
#include "cybsp.h"

cy_stc_usb_usbd_ctxt_t          usbdCtxt;
cy_stc_usb_cal_ctxt_t           hsCalCtxt;
cy_stc_usbss_cal_ctxt_t         ssCalCtxt;
cy_stc_hbdma_context_t          HBW_DrvCtxt;            /* High BandWidth DMA driver context. */
cy_stc_hbdma_dscr_list_t        HBW_DscrList;           /* High BandWidth DMA descriptor free list. */
cy_stc_hbdma_buf_mgr_t          HBW_BufMgr;             /* High BandWidth DMA buffer manager. */
cy_stc_hbdma_mgr_context_t      HBW_MgrCtxt;            /* High BandWidth DMA manager context. */
cy_stc_usb_app_ctxt_t           appCtxt;
DMAC_Type                      *pCpuDmacBase;
DW_Type                        *pCpuDw0Base;
DW_Type                        *pCpuDw1Base;
uint32_t                        hfclkFreq = BCLK__BUS_CLK__HZ;
uint32_t                        dmaClkSel = 0;
bool                            vBusDetDisable = false;
static uint32_t                 g_UsbEvtLogBuf[512u];
volatile bool                   glDeepSleepEnabled = false;

extern uint8_t CyFxUSB20DeviceDscr[];

/* Select SCB interface used for UART based logging. */
#define LOGGING_SCB             (SCB1)
#define LOGGING_SCB_IDX         (1)
#define DEBUG_LEVEL             (3u)

/* Select SCB interface used for I2C command interface. */
#define CONTROL_SCB             (SCB0)
#define CONTROL_SCB_IDX         (0)
#define CONTROL_SCB_INTRSRC     (scb_0_interrupt_IRQn)

/* GPIO to be used as REMOTE-WAKE trigger: P13.0 */
#define REMOTEWAKE_RQT_PORT     (P13_0_PORT)
#define REMOTEWAKE_RQT_PIN      (P13_0_PIN)

/* Product ID used to bind with CyUSB3.sys driver. */
#define USB_PID_CYUSB3          (0x00F1U)

/* Product ID used to bind with WinUsb.sys driver. */
#define USB_PID_WINUSB          (0x4801U)

/* RAM buffer used to hold debug log data. */
#define LOGBUF_RAM_SZ           (1024U)
volatile uint32_t LogDataBuffer[LOGBUF_RAM_SZ / 4U];

/* P4.0 is used for VBus detect functionality. Pin will be low when VBus supply is present. */
#define VBUS_DETECT_GPIO_PORT           (P4_0_PORT)
#define VBUS_DETECT_GPIO_PIN            (P4_0_PIN)
#define VBUS_DETECT_GPIO_INTR           (ioss_interrupts_gpio_dpslp_4_IRQn)
#define VBUS_DETECT_STATE               (0u)

/*
 * Notes on DMA Buffer RAM Usage:
 * 1. The initial part of the buffer RAM is reserved for the descriptors used by the DMA manager.
 *    The space used for this is reserved using the gHbDmaDescriptorSpace array which is placed
 *    in a section named ".hbDmaDescriptor". This array should have a minimum size of 8192 (8 KB)
 *    and has a default size allocation of 16384 bytes (16 KB). No other data should be placed
 *    in this section.
 *
 * 2. The descriptor region is followed by RW data structures which are placed in the ".descSection".
 *    Only data members placed in this section will be initialized during the firmware load
 *    process.
 *
 * 3. The ".descSection" is followed by the ".hbBufSection" which will hold data structures
 *    which do not need to be explicitly initialized (equivalent of ".bss" section).
 *
 * 4. This is followed by the ".hbDmaBufferHeap" section which will be used to allocate all
 *    the DMA buffers from. The gHbDmaBufferHeap array represents the memory region which will
 *    be given to the DMA buffer manager to allocate buffers from and can be sized based on the
 *    available memory. No other data or variables should be placed in this section.
 *
 * Any pre-initialized data which is to be placed in the High BandWidth Buffer RAM should be
 * added to the ".descSection". Any non-initialized data which is to be placed in the High
 * BandWidth Buffer RAM should be added to the ".hbBufSection".
 */

/* Region of 16 KB reserved for High BandWidth DMA descriptors. */
static __attribute__ ((section(".hbDmaDescriptor"), used)) uint32_t gHbDmaDescriptorSpace[16384 / 4];

/* Region of 960 KB reserved for DMA buffer heap. */
/*
 * Note: Since this application requires a large amount of buffer RAM, it is only supported on parts
 * that support 1 MB of DMA buffer RAM.
 */
static __attribute__ ((section(".hbDmaBufferHeap"), used)) uint32_t gHbDmaBufferHeap[960 * 1024 / 4];

bool InitHbDma (void)
{
    cy_en_hbdma_status_t      drvstat;
    cy_en_hbdma_mgr_status_t  mgrstat;

    /* Initialize the HBW DMA driver layer. Only USB DMA adapter is used. */
    drvstat = Cy_HBDma_Init(NULL, USB32DEV, &HBW_DrvCtxt, 0, 0);
    if (drvstat != CY_HBDMA_SUCCESS)
    {
        return false;
    }

    /* Disabling descriptor pre-fetch in USB DMA adapters for this application. */
    HBW_DrvCtxt.USBIN_SCK_GBL->ADAPTER_CONF |= USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_DESCR_PF_EN_N_Msk;
    HBW_DrvCtxt.USBEG_SCK_GBL->ADAPTER_CONF |= USB32DEV_ADAPTER_DMA_SCK_GBL_ADAPTER_CONF_DESCR_PF_EN_N_Msk;

    /* Verify that gHbDmaDescriptorSpace is located at the base of the DMA buffer SRAM. */
    if ((uint32_t)gHbDmaDescriptorSpace != CY_HBW_SRAM_BASE_ADDR) {
        DBG_APP_ERR("High BandWidth DMA descriptors not placed at the correct address\r\n");
        return false;
    }

    /* Setup a HBW DMA descriptor list using the space reserved in gHbDmaDescriptorSpace. */
    mgrstat = Cy_HBDma_DscrList_Create(&HBW_DscrList, sizeof(gHbDmaDescriptorSpace) / 16);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the DMA buffer manager to use the gHbDmaBufferHeap region. */
    mgrstat = Cy_HBDma_BufMgr_Create(&HBW_BufMgr, (uint32_t *)gHbDmaBufferHeap, sizeof(gHbDmaBufferHeap));
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    /* Initialize the HBW DMA channel manager. */
    mgrstat = Cy_HBDma_Mgr_Init(&HBW_MgrCtxt, &HBW_DrvCtxt, &HBW_DscrList, &HBW_BufMgr);
    if (mgrstat != CY_HBDMA_MGR_SUCCESS)
    {
        return false;
    }

    return true;
}

#if FREERTOS_ENABLE

extern void xPortPendSVHandler( void );
extern void xPortSysTickHandler( void );
extern void vPortSVCHandler( void );

void Cy_SysTickIntrWrapper (void)
{
    Cy_USBD_TickIncrement(&usbdCtxt);
    xPortSysTickHandler();
}

/*****************************************************************************
 * Function Name: vPortSetupTimerInterrupt
 *****************************************************************************
 * Summary
 *  Function called by FreeRTOS kernel to start a timer used for task
 *  scheduling. We enable the SysTick interrupt with a period of 1 ms in
 *  this function.
 *
 * Parameters:
 *
 * Return:
 *  void
 ****************************************************************************/
void vPortSetupTimerInterrupt( void )
{
    /* Register the exception vectors. */
    Cy_SysInt_SetVector(PendSV_IRQn, xPortPendSVHandler);
    Cy_SysInt_SetVector(SVCall_IRQn, vPortSVCHandler);
    Cy_SysInt_SetVector(SysTick_IRQn, Cy_SysTickIntrWrapper);

    /* Start the SysTick timer with a period of 1 ms. */
    Cy_SysTick_SetClockSource(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU);
    Cy_SysTick_SetReload(hfclkFreq / 1000U);
    Cy_SysTick_Clear();
    Cy_SysTick_Enable();
}

#else

/*****************************************************************************
 * Function Name: Cy_SysTickIntrWrapper
 *****************************************************************************
 * Summary
 *  Handler for SysTick interrupt.
 *
 * Parameters:
 *
 * Return:
 *  void
 ****************************************************************************/
void Cy_SysTickIntrWrapper (void)
{
    /* If COUNTFLAG is set, update tick count maintained in the USB stack. */
    if (Cy_SysTick_GetCountFlag()) {
        Cy_USBD_TickIncrement(&usbdCtxt);
    }
}

/*****************************************************************************
 * Function Name: SetupTimerInterrupt
 *****************************************************************************
 * Summary
 *  Function to enable the SysTick interrupt to serve as application timer.
 *
 * Parameters:
 *
 * Return:
 *  void
 ****************************************************************************/
void SetupTimerInterrupt (void)
{
    /* Register the exception vectors. */
    Cy_SysInt_SetVector(SysTick_IRQn, Cy_SysTickIntrWrapper);

    /* Start the SysTick timer with a period of 1 ms. */
    Cy_SysTick_SetClockSource(CY_SYSTICK_CLOCK_SOURCE_CLK_CPU);
    Cy_SysTick_SetReload(hfclkFreq / 1000U);
    Cy_SysTick_Clear();
    Cy_SysTick_Enable();
}

volatile uint32_t EpPairWorkPending = 0;
volatile bool     VendorCtrlRqtPending = false;

void Cy_App_MarkEpPairPending (uint8_t lpPairIdx)
{
    uint32_t lock;

    lock = Cy_SysLib_EnterCriticalSection();
    EpPairWorkPending |= (1UL << lpPairIdx);
    Cy_SysLib_ExitCriticalSection(lock);
}

void Cy_App_ClearEpPairPending (uint8_t lpPairIdx)
{
    uint32_t lock;

    lock = Cy_SysLib_EnterCriticalSection();
    EpPairWorkPending &= ~(1UL << lpPairIdx);
    Cy_SysLib_ExitCriticalSection(lock);
}

#endif /* FREERTOS_ENABLE */

/*****************************************************************************
 * Function Name: AddLbkEpDescriptors
 *****************************************************************************
 * Summary
 *  Function to add a pair of loopback endpoints to the configuration
 *  descriptors. No validity check is performed.
 *
 * Parameters:
 *  cy_stc_usb_app_ctxt_t *pAppCtxt: Pointer to application context structure.
 *  cy_stc_app_ep_conf_t  *pEpConf: Pointer to EP pair configuration parameters.
 *
 * Return:
 *  true if descriptor addition was successful, false otherwise.
 ****************************************************************************/
static bool AddLbkEpDescriptors (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_app_ep_conf_t *pEpConf)
{
    uint8_t i;

    if ((pEpConf->outEp == 0) || (pEpConf->outEp >= 16) || (pEpConf->inEp == 0) || (pEpConf->inEp >= 16))
    {
        /* Invalid EP selection. */
        return false;
    }

    if (CyApp_CheckConfigDescriptorLength(pAppCtxt) == false)
    {
        /* Exceeded limit on number of endpoints. */
        return false;
    }

    for (i = 0; i < pAppCtxt->numLbkPairs; i++)
    {
        if ((pEpConf->outEp == pAppCtxt->loopbackInfo[i].OutEndpNum) ||
                (pEpConf->inEp == pAppCtxt->loopbackInfo[i].InEndpNum))
        {
            /* Duplicate endpoint used. */
            return false;
        }
    }

    CyApp_AddEpPairToCfgDscr(pAppCtxt, pEpConf);
    return true;
}

/*******************************************************************************
 * Function name: Cy_Fx3g2_InitPeripheralClocks
 ****************************************************************************//**
 *
 * Function used to enable clocks to different peripherals on the FX10/FX20 device.
 *
 * \param adcClkEnable
 * Whether to enable clock to the ADC in the USBSS block.
 *
 * \param usbfsClkEnable
 * Whether to enable bus reset detect clock input to the USBFS block.
 *
 *******************************************************************************/
void Cy_Fx3g2_InitPeripheralClocks (
        bool adcClkEnable,
        bool usbfsClkEnable)
{
    if (adcClkEnable) {
        /* Divide PERI clock at 75 MHz by 75 to get 1 MHz clock using 16-bit divider #1. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 1, 74);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 1);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_LVDS2USB32SS_CLOCK_SAR, CY_SYSCLK_DIV_16_BIT, 1);
    }

    if (usbfsClkEnable) {
        /* Divide PERI clock at 75 MHz by 750 to get 100 KHz clock using 16-bit divider #2. */
        Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 2, 749);
        Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_16_BIT, 2);
        Cy_SysLib_DelayUs(10U);
        Cy_SysClk_PeriphAssignDivider(PCLK_USB_CLOCK_DEV_BRS, CY_SYSCLK_DIV_16_BIT, 2);
    }

    /* Setting clock for the SCB control channel:
     * Configure PERI 8 bit clock divider for 15 MHz operation and enable it. */
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 0, 4);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 0);
    Cy_SysLib_DelayUs(10U);
    Cy_SysClk_PeriphAssignDivider((en_clk_dst_t)(PCLK_SCB0_CLOCK + CONTROL_SCB_IDX), CY_SYSCLK_DIV_8_BIT, 0);
}

static cy_stc_scb_i2c_context_t  glI2cSlaveCtxt;
static uint8_t glI2cSlaveRdBuffer[32];
static uint8_t glI2cSlaveWrBuffer[32];

/*****************************************************************************
 * Function Name: I2cSlave_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the I2C slave interface used for control commands.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void I2cSlave_ISR(void)
{
    /* Call the interrupt handler provided by the PDL. */
    Cy_SCB_I2C_Interrupt(CONTROL_SCB, &glI2cSlaveCtxt);
}

/*****************************************************************************
* Function Name: SelectEndpointPair
******************************************************************************
* Summary:
*  Enables a pair of OUT and IN endpoints for data loop-back or zero device
*  operation. This function updates the USB descriptors as well as updates
*  the required data structures that are used to handle the data transfers.
*
* Parameters:
*  cy_stc_usb_app_ctxt_t *pAppCtxt: Pointer to application context structure.
*  cy_stc_app_ep_conf_t  *pEpConf: Pointer to EP pair config parameters.
*
* Return:
*  true if the endpoint pair could be initialized, false otherwise.
*****************************************************************************/
static bool SelectEndpointPair(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_app_ep_conf_t *pEpConf)
{
    LoopBackContext_t *pLbkCtxt;
    uint8_t idx;
    bool ret = false;

    /* Make sure parameters are valid. */
    if ((pAppCtxt == NULL) || (pEpConf == NULL) ||
            (pEpConf->numBuff == 0) || (pEpConf->numBuff > MAX_EP_BUF_CNT) ||
            (pEpConf->mPktSizeSS == 0) || (pEpConf->mPktSizeHS == 0) || (pEpConf->mPktSizeFS == 0))
    {
        return ret;
    }

    if (pAppCtxt->numLbkPairs < pAppCtxt->maxLbkPairs)
    {
        pLbkCtxt = &(pAppCtxt->loopbackInfo[pAppCtxt->numLbkPairs]);

        if (AddLbkEpDescriptors(pAppCtxt, pEpConf))
        {
            /* Store the endpoint numbers and increment number of loopback pairs. */
            pLbkCtxt->OutEndpNum = pEpConf->outEp;
            pLbkCtxt->InEndpNum  = pEpConf->inEp;
            pLbkCtxt->LpbkEnable = pEpConf->isLpbk;
            pAppCtxt->numLbkPairs++;

            /* Save the desired count and size of buffers and set all buffer pointers to NULL. */
            pLbkCtxt->bufferCnt  = pEpConf->numBuff;
            pLbkCtxt->bufferSize = pEpConf->buffSize;
            if (pEpConf->buffSize == 0)
                pLbkCtxt->bufferSize = 0x10000UL;

            for (idx = 0; idx < pEpConf->numBuff; idx++)
            {
                pLbkCtxt->outEpBuf[idx] = NULL;
                pLbkCtxt->inEpBuf[idx]  = NULL;
            }

            ret = true;
        }
    }

    return ret;
}

static bool HandleI2cCommand(uint32_t wrLen)
{
    uint32_t command;
    volatile uint32_t *baseAddr;
    bool errorDet = true;
    uint8_t idx;
    cy_stc_app_ep_conf_t epPairConf;

    if (wrLen >= 4)
    {
        command = ((glI2cSlaveRdBuffer[0] << 24u) | (glI2cSlaveRdBuffer[1] << 16u) |
                (glI2cSlaveRdBuffer[2] << 8u) | glI2cSlaveRdBuffer[3]);

        switch (command)
        {
            case 0x434F4E4E: /* CONN: USB connect enable. */
                if (appCtxt.numLbkPairs > 0)
                {
                    DBG_APP_INFO("CONN cmd\r\n");

                    errorDet                  = false;
                    appCtxt.usbConnectEnabled = true;
                    Cy_USB_AppSignalTask(&appCtxt, EV_DEVSTATE_CHG);
                }
                break;

            case 0x44495343: /* DISC: USB disconnect enable */
                if (appCtxt.usbConnectEnabled)
                {
                    DBG_APP_INFO("DISC cmd\r\n");

                    errorDet                  = false;
                    appCtxt.usbConnectEnabled = false;
                    Cy_USB_AppSignalTask(&appCtxt, EV_DEVSTATE_CHG);
                }
                break;

            case 0x4550534C: /* EPSL: Select loop-back EP pair. */
                if ((!appCtxt.usbConnectEnabled) && (wrLen >= 20))
                {
                    epPairConf.isLpbk     = true;
                    epPairConf.outEp      = glI2cSlaveRdBuffer[4];
                    epPairConf.inEp       = glI2cSlaveRdBuffer[5];
                    epPairConf.epType     = (cy_en_usb_endp_type_t)glI2cSlaveRdBuffer[6];
                    epPairConf.mPktSizeSS = (uint16_t)((glI2cSlaveRdBuffer[17] << 8) | glI2cSlaveRdBuffer[16]);
                    epPairConf.mPktSizeHS = (uint16_t)((glI2cSlaveRdBuffer[11] << 8) | glI2cSlaveRdBuffer[10]);
                    epPairConf.mPktSizeFS = (uint16_t)((glI2cSlaveRdBuffer[13] << 8) | glI2cSlaveRdBuffer[12]);
                    epPairConf.pollRate   = glI2cSlaveRdBuffer[14];
                    epPairConf.maxBurst   = glI2cSlaveRdBuffer[18];
                    epPairConf.ssAttrib   = glI2cSlaveRdBuffer[19];
                    epPairConf.numBuff    = glI2cSlaveRdBuffer[7];
                    epPairConf.buffSize   = (uint16_t)((glI2cSlaveRdBuffer[9] << 8) | glI2cSlaveRdBuffer[8]);

                    errorDet = (!SelectEndpointPair(&appCtxt, &epPairConf));
                    if(errorDet) {
                        DBG_APP_INFO("EPSL cmd: error %d\r\n", errorDet);
                    }

                }
                else
                {
                    DBG_APP_ERR("EPSL cmd ignored\r\n");
                }
                break;

            case 0x53534550: /* SSEP: Select source/sink EP pair. */
                if ((!appCtxt.usbConnectEnabled) && (wrLen >= 20))
                {
                    epPairConf.isLpbk     = false;
                    epPairConf.outEp      = glI2cSlaveRdBuffer[4];
                    epPairConf.inEp       = glI2cSlaveRdBuffer[5];
                    epPairConf.epType     = (cy_en_usb_endp_type_t)glI2cSlaveRdBuffer[6];
                    epPairConf.mPktSizeSS = (uint16_t)((glI2cSlaveRdBuffer[17] << 8) | glI2cSlaveRdBuffer[16]);
                    epPairConf.mPktSizeHS = (uint16_t)((glI2cSlaveRdBuffer[11] << 8) | glI2cSlaveRdBuffer[10]);
                    epPairConf.mPktSizeFS = (uint16_t)((glI2cSlaveRdBuffer[13] << 8) | glI2cSlaveRdBuffer[12]);
                    epPairConf.pollRate   = glI2cSlaveRdBuffer[14];
                    epPairConf.maxBurst   = glI2cSlaveRdBuffer[18];
                    epPairConf.ssAttrib   = glI2cSlaveRdBuffer[19];
                    epPairConf.numBuff    = glI2cSlaveRdBuffer[7];
                    epPairConf.buffSize   = (uint16_t)((glI2cSlaveRdBuffer[9] << 8) | glI2cSlaveRdBuffer[8]);

                    errorDet = (!SelectEndpointPair(&appCtxt, &epPairConf));
                    DBG_APP_INFO("SSEP cmd: error %d\r\n", errorDet);
                }
                else
                {
                    DBG_APP_ERR("SSEP cmd ignored\r\n");
                }
                break;

            case 0x434C5250: /* CLRP: Clear all EP pairs. */
                if (!appCtxt.usbConnectEnabled)
                {
                    DBG_APP_INFO("CLRP cmd\r\n");

                    /* Revert descriptors to default values and clear out loopback data structures. */
                    CyApp_RevertConfigDescriptors(&appCtxt);

                    appCtxt.numLbkPairs = 0;
                    memset((uint8_t *)appCtxt.loopbackInfo, 0, appCtxt.maxLbkPairs * sizeof(LoopBackContext_t));

                    errorDet = false;
                }
                break;

            case 0x43594452: /* CYDR: Choose CyUSB3.sys driver. */
                {
                    if (!appCtxt.usbConnectEnabled)
                    {
                        DBG_APP_INFO("CyUSB3 selected\r\n");

                        appCtxt.winusbEnabled = false;
                        CyApp_SetProductId(&appCtxt, USB_PID_CYUSB3);
                        errorDet = false;
                    }
                }
                break;

            case 0x4D534452: /* MSDR: Choose MicroSoft WinUSB driver. */
                {
                    if (!appCtxt.usbConnectEnabled)
                    {
                        DBG_APP_INFO("WinUsb selected\r\n");

                        appCtxt.winusbEnabled = true;
                        CyApp_SetProductId(&appCtxt, USB_PID_WINUSB);
                        errorDet = false;
                    }
                }
                break;

            case 0x52445247: /* RDRG: Dump register content. */
                {
                    if ((wrLen >= 9) && (glI2cSlaveRdBuffer[8] != 0) && (glI2cSlaveRdBuffer[4] == 0x40))
                    {
                        baseAddr = (volatile uint32_t *)((glI2cSlaveRdBuffer[4] << 24u) |
                                (glI2cSlaveRdBuffer[5] << 16u) | (glI2cSlaveRdBuffer[6] << 8u) |
                                glI2cSlaveRdBuffer[7]);
                        DBG_APP_INFO("RegDump\r\n");
                        for (idx = 0; idx < glI2cSlaveRdBuffer[8]; idx++)
                        {
                            DBG_APP_INFO("%x:%x\r\n", (uint32_t)baseAddr, *baseAddr);
                            baseAddr++;
                        }

                        errorDet = false;
                    }
                }
                break;

            case 0x524D5457: /* RMTW: Send Remote Wake Signal. */
                {
                    /* Is USB connection is enabled and remote wake is permitted, signal wake. */
                    if ((appCtxt.usbConnectEnabled) && (Cy_USBD_GetRemoteWakeupStatus(&usbdCtxt)))
                    {
                        DBG_APP_INFO("Send RemoteWake\r\n");
                        Cy_USBD_SignalRemoteWakeup(&usbdCtxt, true);
                        errorDet = false;
                    }
                    else
                    {
                        DBG_APP_WARN("RemoteWake not allowed\r\n");
                    }
                }
                break;

            default:
                DBG_APP_ERR("UNKNOWN cmd\r\n");
                break;
        }
    }

    return errorDet;
}

/*****************************************************************************
 * Function Name: I2cSlave_EvtCb
 *****************************************************************************
 * Summary
 *  Event handler for I2C slave events raised by the PDL.
 *
 * Parameters:
 *  uint32_t event: Type of event callback.
 *
 * Return:
 *  void
 ****************************************************************************/
static void I2cSlave_EvtCb(uint32_t event)
{
    uint32_t wrLen;
    bool     errorDet = true;

    if (event == CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT)
    {
        wrLen = Cy_SCB_I2C_SlaveGetWriteTransferCount(CONTROL_SCB, &glI2cSlaveCtxt);
        if (wrLen == 0)
        {
            return;
        }

        /* Identify and handle the I2C command. */
        errorDet = HandleI2cCommand(wrLen);

        /* Abort and re-start read operation. */
        Cy_SCB_I2C_SlaveAbortWrite(CONTROL_SCB, &glI2cSlaveCtxt);
        Cy_SCB_I2C_SlaveConfigWriteBuf(CONTROL_SCB, glI2cSlaveRdBuffer, 32, &glI2cSlaveCtxt);

        if (errorDet)
        {
            glI2cSlaveWrBuffer[0] = 'E';
            glI2cSlaveWrBuffer[1] = 'R';
            glI2cSlaveWrBuffer[2] = 'R';
            glI2cSlaveWrBuffer[3] = 0;
        }
        else
        {
            glI2cSlaveWrBuffer[0] = 'A';
            glI2cSlaveWrBuffer[1] = 'C';
            glI2cSlaveWrBuffer[2] = 'K';
            glI2cSlaveWrBuffer[3] = 0;
        }

        /* Send the response. */
        Cy_SCB_I2C_SlaveConfigReadBuf(CONTROL_SCB, glI2cSlaveWrBuffer, 4, &glI2cSlaveCtxt);
    }
}

/*****************************************************************************
 * Function Name: VbusDetGpio_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the Vbus detect GPIO transition detection.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void VbusDetGpio_ISR(void)
{
    /* Remember that VBus change has happened and disable the interrupt. */
    appCtxt.vbusChangeIntr = true;
    Cy_USBD_AddEvtToLog(&usbdCtxt, CY_SSCAL_EVT_VBUS_CHG_INTR);
    Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 0);
    Cy_USB_AppSignalTask(&appCtxt, EV_DEVSTATE_CHG);
}

#if (!CY_CPU_CORTEX_M4)

#if REMOTEWAKE_EN
static void RemoteWakeGpio_ISR(void);
#endif /* REMOTEWAKE_EN */

/*****************************************************************************
 * Function Name: Cy_Gpio_SCB_Common_ISR
 *****************************************************************************
 * Summary
 *  Common interrupt handler for Control SCB and GPIO interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void Cy_Gpio_SCB_Common_ISR (void)
{
    if (Cy_SCB_GetInterruptCause(CONTROL_SCB) != 0) {
        I2cSlave_ISR();
    }

    if (Cy_GPIO_GetInterruptStatusMasked(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) != 0) {
        VbusDetGpio_ISR();
    }

#if REMOTEWAKE_EN
    if (Cy_GPIO_GetInterruptStatusMasked(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN) != 0) {
        RemoteWakeGpio_ISR();
    }
#endif /* REMOTEWAKE_EN */
}
#endif /* (!CY_CPU_CORTEX_M4) */

#if REMOTEWAKE_EN

static volatile bool gl_RemoteWakeDone = false;

/*****************************************************************************
 * Function Name: InitiateRemoteWake
 *****************************************************************************
 * Summary
 *  Function to initiate remote wake signalling.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void InitiateRemoteWake (void)
{
    cy_en_usbss_lnk_power_mode_t curLinkState;

    if (!gl_RemoteWakeDone) {
        DBG_APP_INFO("Remote wake request detected\r\n");
        gl_RemoteWakeDone = true;

        if (appCtxt.devSpeed > CY_USBD_USB_DEV_HS) {
            Cy_USBSS_Cal_GetLinkPowerState(&ssCalCtxt, &curLinkState);
            /* Initiate remote wake-up. If successful, schedule sending of DEV_NOTIFICATION TP. */
            if (
                    (curLinkState == CY_USBSS_LPM_U3) &&
                    (appCtxt.functionWakeEnable) &&
                    (Cy_USBD_GetUSBLinkActive(appCtxt.pUsbdCtxt) == CY_USBD_STATUS_SUCCESS)
               ) {
                Cy_USBD_SendSSDeviceNotification(appCtxt.pUsbdCtxt, CY_USBD_NOTIF_FUNC_WAKE, 0, 0);
            }
        } else {
            if (appCtxt.pUsbdCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) {
                /* If link is in suspend, send remote wake. Duration of signal will be taken care by HS_CAL */
                Cy_USBD_SignalRemoteWakeup(appCtxt.pUsbdCtxt, true);
            }
        }
    }
}

/*****************************************************************************
 * Function Name: RemoteWakeGpio_ISR
 *****************************************************************************
 * Summary
 *  Interrupt handler for the remote wake-up trigger GPIO.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
static void RemoteWakeGpio_ISR(void)
{
    uint32_t state1, state2;
    uint32_t debounce_count = 0;

    /* Clear the GPIO interrupt. */
    Cy_GPIO_ClearInterrupt(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN);
    state1 = Cy_GPIO_Read(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN);

    /* Debounce by confirming consistent GPIO state for more than 300 us. */
    do {
        Cy_SysLib_DelayUs(100UL);
        state2 = Cy_GPIO_Read(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN);
        if (state2 == state1) {
            debounce_count++;
        } else {
            state1 = state2;
            debounce_count = 0;
        }
    } while (debounce_count < 3);

    /* Clear the GPIO interrupt once again. */
    Cy_GPIO_ClearInterrupt(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN);

    /* Initiate remote wake signalling. */
    InitiateRemoteWake();
}

static void ConfigureRemoteWakeTriggerPin (void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_sysint_t          intrCfg;

    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure GPIO pin as interrupt source to trigger remote wakeup. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_RISING;
    pinCfg.intMask   = 0x01UL;
    Cy_GPIO_Pin_Init(REMOTEWAKE_RQT_PORT, REMOTEWAKE_RQT_PIN, &pinCfg);

#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = ioss_interrupts_gpio_dpslp_13_IRQn;
    intrCfg.intrPriority = 7;
    Cy_SysInt_Init(&intrCfg, RemoteWakeGpio_ISR);
#else
    intrCfg.cm0pSrc = ioss_interrupts_gpio_dpslp_13_IRQn;
    intrCfg.intrSrc = NvicMux4_IRQn;
    intrCfg.intrPriority = 3;
    Cy_SysInt_Init(&intrCfg, Cy_Gpio_SCB_Common_ISR);
#endif
    NVIC_EnableIRQ(intrCfg.intrSrc);
}

#endif /* REMOTEWAKE_EN */

/*******************************************************************************
 * Function name: Cy_Fx3G2_ConfigureIOs
 ****************************************************************************//**
 *
 * Function used to configure common GPIOs used across all FX10 applications.
 * The following pins are configured in strong drive mode:
 *  P0.0, P0.1   : Used for debug status indication during USB link bring-up
 *  P11.0, P11.1 : Chip level Digital Design For Test pins used for debugging.
 *  P9.2, P9.3   : USB Design For Test GPIOs used for debugging.
 *
 * \param dftEnable
 * Whether the Design For Test pins should be configured for debug function.
 *
 *******************************************************************************/
void Cy_Fx3G2_ConfigureIOs (
        bool dftEnable)
{
    cy_stc_gpio_pin_config_t pinCfg;

    /* Clear the structure to start with. */
    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    /* Configure P0.0 and P0.1 as General Purpose Output pins
     * with strong output drivers enabled.
     */
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom     = P0_0_GPIO;
    Cy_GPIO_Pin_Init(P0_0_PORT, P0_0_PIN, &pinCfg);
    pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    pinCfg.hsiom     = P0_1_GPIO;
    Cy_GPIO_Pin_Init(P0_1_PORT, P0_1_PIN, &pinCfg);

    if (dftEnable) {
        /* Choose chip level DDFT function for P11.0 and P11.1 pins
         * and enable output drivers.
         */
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_0_SRSS_DDFT_PIN_IN0;
        Cy_GPIO_Pin_Init(P11_0_PORT, P11_0_PIN, &pinCfg);
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P11_1_SRSS_DDFT_PIN_IN1;
        Cy_GPIO_Pin_Init(P11_1_PORT, P11_1_PIN, &pinCfg);

        /* Choose HBWSS GPIO DFT function for P9.2 and P9.3 pins and
         * enable output drivers.
         */
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P9_2_LVDS2USB32SS_USB32_GPIO_DDFT_O0;
        Cy_GPIO_Pin_Init(P9_2_PORT, P9_2_PIN, &pinCfg);
        pinCfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
        pinCfg.hsiom     = P9_3_LVDS2USB32SS_USB32_GPIO_DDFT_O1;
        Cy_GPIO_Pin_Init(P9_3_PORT, P9_3_PIN, &pinCfg);

        /* Route the DDFT outputs from the High BandWidth SubSystem to
         * the chip level pins.
         */
        SRSS_TST_DDFT_FAST_CTL_REG = 0x00000C0BUL;
        SRSS_TST_DDFT_SLOW_CTL_REG = 0xC0008080UL;
    }
}

/*****************************************************************************
 * Function Name: InitSCBs
 *****************************************************************************
 * Summary
 *  Initialize the SCB interfaces used for logging as a UART block and as I2C
 *  slave to receive commands.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void InitSCBs (void)
{
    cy_stc_gpio_pin_config_t pinCfg;
    cy_stc_scb_i2c_config_t  i2cCfg;
    cy_stc_sysint_t          intrCfg;
    cy_stc_debug_config_t    dbgCfg;

#if (!USBFS_LOGS_ENABLE)
    /* Initialize SCB for debug log output. */
    InitUart(LOGGING_SCB_IDX);
#endif /* (!USBFS_LOGS_ENABLE) */

    /* Enable clocks for all peripherals that are used. */
    Cy_Fx3g2_InitPeripheralClocks(true, true);

    dbgCfg.pBuffer   = (uint8_t *)LogDataBuffer;
    dbgCfg.traceLvl  = DEBUG_LEVEL;
    dbgCfg.bufSize   = LOGBUF_RAM_SZ;
#if USBFS_LOGS_ENABLE
    dbgCfg.dbgIntfce = CY_DEBUG_INTFCE_USBFS_CDC;
#else
    dbgCfg.dbgIntfce = CY_DEBUG_INTFCE_UART_SCB1;
#endif /* USBFS_LOGS_ENABLE */
    dbgCfg.printNow  = true;                    /* Printing messages immediately for now. */
    Cy_Debug_LogInit(&dbgCfg);

    /* Configure SCB as I2C slave. */
    i2cCfg.i2cMode             = CY_SCB_I2C_SLAVE;
    i2cCfg.useRxFifo           = false;
    i2cCfg.useTxFifo           = false;
    i2cCfg.slaveAddress        = 0x08u;
    i2cCfg.slaveAddressMask    = 0xFEu;
    i2cCfg.acceptAddrInFifo    = false;
    i2cCfg.ackGeneralAddr      = false;
    i2cCfg.enableWakeFromSleep = false;
    i2cCfg.enableDigitalFilter = false;
    i2cCfg.lowPhaseDutyCycle   = 7u;
    i2cCfg.highPhaseDutyCycle  = 5u;
    Cy_SCB_I2C_Init(CONTROL_SCB, &i2cCfg, &glI2cSlaveCtxt);

    /* Configure SCB pins for I2C operation. */
    memset ((void *)&pinCfg, 0, sizeof(pinCfg));

    pinCfg.driveMode = CY_GPIO_DM_OD_DRIVESLOW;
    pinCfg.hsiom     = P10_0_SCB0_I2C_SCL;
    Cy_GPIO_Pin_Init(P10_0_PORT, P10_0_PIN, &pinCfg);
    pinCfg.hsiom     = P10_1_SCB0_I2C_SDA;
    Cy_GPIO_Pin_Init(P10_1_PORT, P10_1_PIN, &pinCfg);

    /* Configure the pins used for debugging. */
    Cy_Fx3G2_ConfigureIOs(true);

    /* Configure VBus detect GPIO. Leave interrupt disabled at present. */
    pinCfg.driveMode = CY_GPIO_DM_HIGHZ;
    pinCfg.hsiom     = HSIOM_SEL_GPIO;
    pinCfg.intEdge   = CY_GPIO_INTR_BOTH;
    pinCfg.intMask   = 0x00UL;
    Cy_GPIO_Pin_Init(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, &pinCfg);

    /* Register edge detect interrupt for Vbus detect GPIO. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrPriority = 7;
    Cy_SysInt_Init(&intrCfg, VbusDetGpio_ISR);
#else
    intrCfg.cm0pSrc = VBUS_DETECT_GPIO_INTR;
    intrCfg.intrSrc = NvicMux4_IRQn;
    intrCfg.intrPriority = 3;
    Cy_SysInt_Init(&intrCfg, Cy_Gpio_SCB_Common_ISR);
#endif
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register I2C event callback. */
    Cy_SCB_I2C_RegisterEventCallback(CONTROL_SCB, I2cSlave_EvtCb, &glI2cSlaveCtxt);

    /* Register interrupt handler for SCB-I2C. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc = CONTROL_SCB_INTRSRC;
    intrCfg.intrPriority = 7;
    Cy_SysInt_Init(&intrCfg, I2cSlave_ISR);
#else
    intrCfg.cm0pSrc = CONTROL_SCB_INTRSRC;
    intrCfg.intrSrc = NvicMux4_IRQn;
    intrCfg.intrPriority = 3;
    Cy_SysInt_Init(&intrCfg, Cy_Gpio_SCB_Common_ISR);
#endif
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Enable the SCB for operation. */
    Cy_SCB_I2C_Enable(CONTROL_SCB);
}

/*****************************************************************************
 * Function Name: PrintVersionInfo
 ******************************************************************************
 * Summary:
 *  Function to print version information to UART console.
 *
 * Parameters:
 *  type: Type of version string.
 *  typeLen: Length of version type string.
 *  vMajor: Major version number (0 - 99)
 *  vMinor: Minor version number (0 - 99)
 *  vPatch: Patch version number (0 - 99)
 *  vBuild: Build number (0 - 9999)
 *
 * Return:
 *  None
 *****************************************************************************/
void PrintVersionInfo (const char *type, uint8_t typeLen,
                       uint8_t vMajor, uint8_t vMinor, uint8_t vPatch, uint16_t vBuild)
{
    char tString[32];

    memcpy(tString, type, typeLen);
    tString[typeLen++] = '0' + (vMajor / 10);
    tString[typeLen++] = '0' + (vMajor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vMinor / 10);
    tString[typeLen++] = '0' + (vMinor % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vPatch / 10);
    tString[typeLen++] = '0' + (vPatch % 10);
    tString[typeLen++] = '.';
    tString[typeLen++] = '0' + (vBuild / 1000);
    tString[typeLen++] = '0' + ((vBuild % 1000) / 100);
    tString[typeLen++] = '0' + ((vBuild % 100) / 10);
    tString[typeLen++] = '0' + (vBuild % 10);
    tString[typeLen++] = '\r';
    tString[typeLen++] = '\n';
    tString[typeLen]   = 0;

    DBG_APP_INFO("%s", tString);
}

/*****************************************************************************
 * Function Name: UsbHS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-HS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void UsbHS_ISR(void)
{
#if FREERTOS_ENABLE
    if (Cy_USBHS_Cal_IntrHandler(&hsCalCtxt))
    {
        portYIELD_FROM_ISR(true);
    }
#else
    Cy_USBHS_Cal_IntrHandler(&hsCalCtxt);
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: UsbSS_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB-SS Interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void UsbSS_ISR(void)
{
    Cy_USBSS_Cal_IntrHandler(&ssCalCtxt);

#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: DMAC_Chn1_ISR
 ******************************************************************************
 * Summary:
 *  Handler for interrupts on DMAC channel 1 which is used for EP0 out
 *  data transfers.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void DMAC_Chn1_ISR (void)
{
    Cy_USBD_EP0OutDma_IntrHandler(&usbdCtxt);

#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: UsbIngressDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB Ingress DMA adapter interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void UsbIngressDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_IN);

#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: UsbEgressDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for USB Egress DMA adapter interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *****************************************************************************/
void UsbEgressDma_ISR(void)
{
    /* Call the HBDMA interrupt handler with the appropriate adapter ID. */
    Cy_HBDma_HandleInterrupts(&HBW_DrvCtxt, CY_HBDMA_ADAP_USB_EG);

#if FREERTOS_ENABLE
    portYIELD_FROM_ISR(true);
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: UsbDevInit
 *****************************************************************************
 * Summary
 *  Initialize USB device block and interrupts.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  void
 ****************************************************************************/
void UsbDevInit (void)
{
    const char start_string[] = "***** FX20: USB Test App *****\r\n";
    cy_stc_sysint_t intrCfg;

    /* Initialize the UART for logging and the I2C slave. */
    InitSCBs ();

    /* Send a start-up string using the logging SCB. */
    DBG_APP_INFO("%s", start_string);

    /* Print application and USBD stack version information. */
    PrintVersionInfo("APP_VERSION: ", 13, APP_VERSION_MAJOR, APP_VERSION_MINOR,
            APP_VERSION_PATCH, APP_VERSION_BUILD);
    PrintVersionInfo("USBD_VERSION: ", 14, USBD_VERSION_MAJOR, USBD_VERSION_MINOR,
            USBD_VERSION_PATCH, USBD_VERSION_BUILD);
    PrintVersionInfo("HBDMA_VERSION: ", 15, HBDMA_VERSION_MAJOR, HBDMA_VERSION_MINOR,
            HBDMA_VERSION_PATCH, HBDMA_VERSION_BUILD);

    /* Register ISR for and enable USBHS Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_active_o_IRQn;
    intrCfg.intrSrc      = NvicMux3_IRQn;
    intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, UsbHS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.cm0pSrc      = usbhsdev_interrupt_u2d_dpslp_o_IRQn;
    intrCfg.intrSrc      = NvicMux3_IRQn;
    intrCfg.intrPriority = 2;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, UsbHS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register ISR for and enable USBSS Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc      = NvicMux0_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &UsbSS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Map the USB32 wakeup interrupt to the same ISR. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 4;
#else
    intrCfg.intrSrc      = NvicMux0_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_wakeup_int_o_IRQn;
    intrCfg.intrPriority = 0;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &UsbSS_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register ISR for and enable USBSS Ingress DMA Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc      = NvicMux1_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 1;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &UsbIngressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register ISR for and enable USBSS Egress DMA Interrupt. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 3;
#else
    intrCfg.intrSrc      = NvicMux2_IRQn;
    intrCfg.cm0pSrc      = lvds2usb32ss_usb32_egrs_dma_int_o_IRQn;
    intrCfg.intrPriority = 1;
#endif /* CY_CPU_CORTEX_M4 */
    Cy_SysInt_Init(&intrCfg, &UsbEgressDma_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);

    /* Register ISR for and enable DMAC channel 1 interrupt only for CM4 case. */
#if CY_CPU_CORTEX_M4
    intrCfg.intrSrc      = cpuss_interrupts_dmac_1_IRQn;
    intrCfg.intrPriority = 4;
    Cy_SysInt_Init(&intrCfg, &DMAC_Chn1_ISR);
    NVIC_EnableIRQ(intrCfg.intrSrc);
#endif /* CY_CPU_CORTEX_M4 */

    /* Set the desired USB connection type based on build parameter. */
    appCtxt.desiredSpeed = USB_CONN_TYPE;
}

/*****************************************************************************
 * Function Name: OutEpDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DMA transfer completion on OUT endpoint.
 *
 * Parameters:
 *  endpNumber: OUT endpoint number.
 *
 * Return:
 *  None
 *****************************************************************************/
void OutEpDma_ISR(uint8_t endpNumber)
{
    uint8_t i;
    bool yield = false;

    /* Clear the interrupt. */
    Cy_USB_AppClearDmaInterrupt(&appCtxt, endpNumber, CY_USB_ENDP_DIR_OUT);

    /* Find the loopback pair corresponding to this endpoint and mark OUT DMA done. */
    for (i = 0; i < appCtxt.numLbkPairs; i++)
    {
        if (appCtxt.loopbackInfo[i].OutEndpNum == endpNumber)
        {
            appCtxt.loopbackInfo[i].BulkOutDmaDone = true;
#if FREERTOS_ENABLE
            yield |= Cy_USB_AppSignalTask(&appCtxt, (1U << (i + 1)));
#else
            Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
        }
    }

#if FREERTOS_ENABLE
    /* Yield control to the application task immediately. */
    portYIELD_FROM_ISR(yield);
#else
    (void)yield;
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
 * Function Name: InEpDma_ISR
 ******************************************************************************
 * Summary:
 *  Handler for DMA transfer completion on IN endpoint.
 *
 * Parameters:
 *  endpNumber: IN endpoint number.
 *
 * Return:
 *  None
 *****************************************************************************/
void InEpDma_ISR(uint8_t endpNumber)
{
    uint8_t i;
    bool yield = false;

    /* Clear the interrupt. */
    Cy_USB_AppClearDmaInterrupt(&appCtxt, endpNumber, CY_USB_ENDP_DIR_IN);

    /* Find the loopback pair corresponding to this endpoint and mark IN DMA done. */
    for (i = 0; i < appCtxt.numLbkPairs; i++)
    {
        if (appCtxt.loopbackInfo[i].InEndpNum == endpNumber)
        {
            appCtxt.loopbackInfo[i].BulkInDmaDone = true;
#if FREERTOS_ENABLE
            yield |= Cy_USB_AppSignalTask(&appCtxt, (1U << (i + 1)));
#else
            Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
        }
    }

#if FREERTOS_ENABLE
    /* Yield control to the application task immediately. */
    portYIELD_FROM_ISR(yield);
#else
    (void)yield;
#endif /* FREERTOS_ENABLE */
}

/*****************************************************************************
* Function Name: ResetLoopBackCtxt
******************************************************************************
* Summary:
*  Resets the state variables in the loop-back context structure when the device
*  is not in configured state.
*
* Parameters
*  pAppCtxt: Pointer to application context structure.
*
* Return:
*  void
*****************************************************************************/
void ResetLoopBackCtxt(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t i;
    LoopBackContext_t *plbkctxt;

    /* Clear state variables and disable DMA transfer interrupts. */
    for (i = 0; i < pAppCtxt->numLbkPairs; i++)
    {
        plbkctxt = &(pAppCtxt->loopbackInfo[i]);
        plbkctxt->packetCnt      = 0;
        plbkctxt->nextRdIdx      = 0;
        plbkctxt->nextWrIdx      = 0;
        plbkctxt->BulkOutDmaDone = false;
        plbkctxt->BulkInDmaDone  = false;
        plbkctxt->rdQueued       = false;
        plbkctxt->wrQueued       = false;
        plbkctxt->zlpRcvd        = false;
        plbkctxt->slpRcvd        = false;
        plbkctxt->slpLen         = 0;
        Cy_USB_AppInitDmaIntr(pAppCtxt, plbkctxt->OutEndpNum, CY_USB_ENDP_DIR_OUT, NULL);
        Cy_USB_AppInitDmaIntr(pAppCtxt, plbkctxt->InEndpNum, CY_USB_ENDP_DIR_IN, NULL);
    }
}

/*****************************************************************************
* Function Name: HandleLoopBackTask
******************************************************************************
* Summary:
*  Main function which handles the data loop-back operation on a pair of OUT
*  and IN endpoints. This function queues reads on the OUT endpoints and once
*  the read has returned, queues writes of the same data on the IN endpoint.
*
* Parameters:
*  cy_stc_usb_app_ctxt_t *pAppCtxt: USB application context.
*  uint8_t lpPairIdx: Index of the loop-back endpoint pair (0 to numLbkPairs - 1).

* Return:
*  true if the function should be called again, false otherwise.
*****************************************************************************/
bool HandleLoopBackTask(cy_stc_usb_app_ctxt_t *pAppCtxt, uint8_t lpPairIdx)
{
    LoopBackContext_t *pCtxt = &(pAppCtxt->loopbackInfo[lpPairIdx]);
    uint16_t currentLength;
    bool workPending = false;
    uint32_t lock;

    /* In SS connections, all source/sink DMA handling is done in the DMA callbacks. */
    if ((pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) && (!pCtxt->LpbkEnable))
    {
        return workPending;
    }

    /* If we have a pending read operation, check whether it has been completed. */
    if (pCtxt->rdQueued)
    {
        /* Handle ZLP event only if DMA completion is not present. */
        if ((pCtxt->zlpRcvd) && (pCtxt->BulkOutDmaDone == false))
        {
            lock = Cy_SysLib_EnterCriticalSection();
            /* Disable the DMA channel and calculate data received in this buffer so far. */
            pCtxt->packetLen[pCtxt->nextRdIdx] = Cy_USB_AppReadShortPacket(pAppCtxt, pCtxt->OutEndpNum, 0);

            /* Mark DMA done and clear the ZLP received flag. */
            pCtxt->BulkOutDmaDone = true;
            pCtxt->zlpRcvd = false;

            /* Clear and re-enable the ZLP interrupt. */
            Cy_USBD_ClearZlpSlpIntrEnableMask(&usbdCtxt, pCtxt->OutEndpNum, CY_USB_ENDP_DIR_OUT, true);
            Cy_SysLib_ExitCriticalSection(lock);
        }

        /* Handle SLP event only if DMA completion is not present. */
        if ((pCtxt->slpRcvd) && (pCtxt->BulkOutDmaDone == false))
        {
            lock = Cy_SysLib_EnterCriticalSection();

            /* Disable the DMA channel and reconfigure for the actual data size. */
            pCtxt->packetLen[pCtxt->nextRdIdx] = Cy_USB_AppReadShortPacket(pAppCtxt, pCtxt->OutEndpNum, pCtxt->slpLen);

            /* Clear SLP related status. */
            pCtxt->slpRcvd = false;
            pCtxt->slpLen  = 0;

            /* Assert the trigger output from Ingress EPM to DMA channel. */
            if (pAppCtxt->numLbkPairs <= 2) {
                Cy_TrigMux_SwTrigger(TRIG_IN_MUX_5_USBHSDEV_TR_OUT0 + pCtxt->OutEndpNum,
                        CY_TRIGGER_TWO_CYCLES);
            } else {
                Cy_TrigMux_SwTrigger(TRIG_IN_MUX_0_USBHSDEV_TR_OUT0 + pCtxt->OutEndpNum,
                        CY_TRIGGER_TWO_CYCLES);
            }

            Cy_SysLib_ExitCriticalSection(lock);
        }

        /* If read DMA transfer is completed, update state variables. */
        lock = Cy_SysLib_EnterCriticalSection();
        if (pCtxt->BulkOutDmaDone)
        {
            pCtxt->BulkOutDmaDone = false;

            if (pCtxt->LpbkEnable)
            {
                /* In case of loop-back operation, increment number of data buffers filled up. */
                pCtxt->packetCnt++;
            }

            /* Update next read buffer index. */
            pCtxt->nextRdIdx++;
            if (pCtxt->nextRdIdx >= pCtxt->bufferCnt)
            {
                pCtxt->nextRdIdx = 0;
            }

            /* Read has been completed. */
            pCtxt->rdQueued = false;
            workPending = true;
        }
        Cy_SysLib_ExitCriticalSection(lock);
    }

    /* If no read is pending, queue one. */
    if (!(pCtxt->rdQueued))
    {
        lock = Cy_SysLib_EnterCriticalSection();
        if (pCtxt->packetCnt < pCtxt->bufferCnt)
        {
            /* Queue the next read operation. */
            if ((pCtxt->LpbkEnable) && (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS)) {
                pCtxt->packetLen[pCtxt->nextRdIdx] = Cy_USB_AppGetMaxPktSize (pAppCtxt, pCtxt->OutEndpNum, CY_USB_ENDP_DIR_OUT);
            } else {
                pCtxt->packetLen[pCtxt->nextRdIdx] = pCtxt->bufferSize;
            }
            Cy_USB_AppQueueRead(pAppCtxt, pCtxt->OutEndpNum, (uint8_t *)(pCtxt->outEpBuf[pCtxt->nextRdIdx]),
                    pCtxt->packetLen[pCtxt->nextRdIdx]);

            /* Set flag to wait until read is completed. */
            pCtxt->rdQueued = true;
        }
        Cy_SysLib_ExitCriticalSection(lock);
    }

    if (pCtxt->wrQueued)
    {
        lock = Cy_SysLib_EnterCriticalSection();

        /* If write DMA transfer is completed, update state variables. */
        if (pCtxt->BulkInDmaDone)
        {
            pCtxt->BulkInDmaDone = false;

            if (pCtxt->LpbkEnable)
            {
                /* In case of loop back operation, decrement number of pending data buffers. */
                pCtxt->packetCnt--;
            }

            /* Update next write buffer index. */
            pCtxt->nextWrIdx++;
            if (pCtxt->nextWrIdx >= pCtxt->bufferCnt)
            {
                pCtxt->nextWrIdx = 0;
            }

            /* Write has been completed. */
            pCtxt->wrQueued = false;
            workPending = true;
        }

        Cy_SysLib_ExitCriticalSection(lock);
    }

    /* If no write is pending and data is available, queue one. */
    if (!(pCtxt->wrQueued))
    {
        lock = Cy_SysLib_EnterCriticalSection();

        if ((!pCtxt->LpbkEnable) || (pCtxt->packetCnt > 0))
        {
            currentLength = pCtxt->packetLen[pCtxt->nextWrIdx];
            if (!pCtxt->LpbkEnable)
            {
                currentLength = pCtxt->bufferSize;
            }

            if ((usbdCtxt.devSpeed <= CY_USBD_USB_DEV_HS) && (currentLength == 0))
            {
                /* Queue a ZLP on IN endpoint. IP should take care of doing this after EPM is empty. */
                Cy_USBD_SendEgressZLP(&usbdCtxt, pCtxt->InEndpNum);

                /* Wait until ZLP sending is complete. */
                pCtxt->wrQueued = true;
            }
            else
            {
                /* Loop the data back on IN endpoint. */
                Cy_USB_AppQueueWrite(pAppCtxt, pCtxt->InEndpNum,
                        (uint8_t *)(pCtxt->inEpBuf[pCtxt->nextWrIdx]), currentLength);

                /* Wait until write is complete. */
                pCtxt->wrQueued = true;
            }
        }

        Cy_SysLib_ExitCriticalSection(lock);
    }

#if (!FREERTOS_ENABLE)
    lock = Cy_SysLib_EnterCriticalSection();
    if (
            (!workPending) &&
            ((pCtxt->rdQueued) || (pCtxt->packetCnt >= pCtxt->bufferCnt)) &&
            ((pCtxt->wrQueued) || (pCtxt->packetCnt == 0)) &&
            (!pCtxt->BulkOutDmaDone) && (!pCtxt->slpRcvd) && (!pCtxt->zlpRcvd) && (!pCtxt->BulkInDmaDone)
       ) {
        Cy_App_ClearEpPairPending(lpPairIdx);
    }
    Cy_SysLib_ExitCriticalSection(lock);
#endif /* FREERTOS_ENABLE */

    return workPending;
}

cy_israddress GetEPInDmaIsr(uint8_t epNum);
cy_israddress GetEPOutDmaIsr(uint8_t epNum);

/*****************************************************************************
* Function Name: EnableEpDmaISRs
******************************************************************************
* Summary:
*  Registers Interrupt Service Routines for the DMA (DW) DMA channels associated
*  with the OUT and IN endpoints.
*
* Parameters:
*  pAppCtxt: Pointer to application context structure.

* Return:
*  void
*****************************************************************************/
static void EnableEpDmaISRs(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t i;
    uint8_t outEp, inEp;

    /* Register ISRs for the loopback EP pairs. */
    for (i = 0; i < pAppCtxt->numLbkPairs; i++)
    {
        outEp = pAppCtxt->loopbackInfo[i].OutEndpNum;
        inEp  = pAppCtxt->loopbackInfo[i].InEndpNum;
        Cy_USB_AppInitDmaIntr(pAppCtxt, outEp, CY_USB_ENDP_DIR_OUT, GetEPOutDmaIsr(outEp));
        Cy_USB_AppInitDmaIntr(pAppCtxt, inEp, CY_USB_ENDP_DIR_IN, GetEPInDmaIsr(inEp));

#if (!FREERTOS_ENABLE)
        Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
    }
}

/*****************************************************************************
* Function Name: LoopBackApp_ZLPCallback
******************************************************************************
* Summary:
*  Callback function providing notification when a ZLP is received on OUT
*  endpoint.
*
* Parameters:
*  pUsbApp: Application context pointer.
*  pUsbdCtxt: USBD context pointer.
*  pMsg: Provides details of endpoint where ZLP was received.

* Return:
*  void
*****************************************************************************/
void
LoopBackApp_ZLPCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;
    uint8_t endpNumber = (uint8_t)(pMsg->data[0] & 0x7FU);
    uint8_t i;

    /* Look-up the loopback pair corresponding to this endpoint. */
    for (i = 0; i < pAppCtxt->numLbkPairs; i++)
    {
        if (pMsg->type == CY_USB_CAL_MSG_OUT_ZLP)
        {
            /* Check if ZLP received on the Ingress (OUT) endpoint. */
            if (pAppCtxt->loopbackInfo[i].OutEndpNum == endpNumber)
            {
                pAppCtxt->loopbackInfo[i].zlpRcvd = true;
#if FREERTOS_ENABLE
                Cy_USB_AppSignalTask(pAppCtxt, (1U << (i + 1)));
#else
                Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
                break;
            }
        }
        else
        {
            /* Check if ZLP send finished on the Egress (IN) endpoint. */
            if (pAppCtxt->loopbackInfo[i].InEndpNum == endpNumber)
            {
                pAppCtxt->loopbackInfo[i].BulkInDmaDone = true;
#if FREERTOS_ENABLE
                Cy_USB_AppSignalTask(pAppCtxt, (1U << (i + 1)));
#else
                Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
                break;
            }
        }
    }
}

/*****************************************************************************
* Function Name: LoopBackApp_SLPCallback
******************************************************************************
* Summary:
*  Callback function providing notification when a SLP is received on OUT
*  endpoint.
*
* Parameters:
*  pUsbApp: Application context pointer.
*  pUsbdCtxt: USBD context pointer.
*  pMsg: Provides details of endpoint and size of SLP received.

* Return:
*  void
*****************************************************************************/
void
LoopBackApp_SLPCallback (void *pUsbApp, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                       cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pUsbApp;
    uint8_t endpNumber = (uint8_t)(pMsg->data[0] & 0x7FU);
    uint8_t i;

    /* Look-up the loopback pair corresponding to this endpoint. */
    for (i = 0; i < pAppCtxt->numLbkPairs; i++)
    {
        if (pAppCtxt->loopbackInfo[i].OutEndpNum == endpNumber)
        {
            pAppCtxt->loopbackInfo[i].slpRcvd = true;
            pAppCtxt->loopbackInfo[i].slpLen  = (uint16_t)pMsg->data[1];
#if FREERTOS_ENABLE
            Cy_USB_AppSignalTask(pAppCtxt, (1U << (i + 1)));
#else
            Cy_App_MarkEpPairPending(i);
#endif /* FREERTOS_ENABLE */
            break;
        }
    }
}

/*****************************************************************************
* Function Name: GetUsbDeviceConfiguration
******************************************************************************
* Summary:
*  Get USB device configuration data through UART and configures device
*  operation accordingly. This function waits until a connect command is
*  received over the UART.
*
* Parameters:
*  cy_stc_usb_app_ctxt_t *pAppCtxt: Pointer to application context structure.

* Return:
*  None
*****************************************************************************/
static void GetUsbDeviceConfiguration(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    uint8_t lastEpIndex;

    Cy_SCB_I2C_SlaveConfigWriteBuf(CONTROL_SCB, glI2cSlaveRdBuffer, 32, &glI2cSlaveCtxt);

#if USE_DFLT_CONFIG
    cy_stc_testdev_config_t *pTestCfg = (cy_stc_testdev_config_t *)TESTDEV_CFG_STRUCT_ADDRESS;
    uint8_t epIdx;

    if ((pTestCfg->startSig == TESTDEV_CFG_START_SIG) && (pTestCfg->endSig == TESTDEV_CFG_END_SIG)) {
        /* Set the default device configuration based on the test config data provided in RAM. */
        switch(pTestCfg->connSpeed) {
            case 0:
                pAppCtxt->desiredSpeed = CY_USBD_USB_DEV_SS_GEN1;
                break;

            case 1:
                pAppCtxt->desiredSpeed = CY_USBD_USB_DEV_SS_GEN2;
                break;

            case 2:
                pAppCtxt->desiredSpeed = CY_USBD_USB_DEV_HS;
                break;

            case 3:
                pAppCtxt->desiredSpeed = CY_USBD_USB_DEV_FS;

                /* Disable L1 support in USBFS case. */
                CyFxUSB20DeviceDscr[2] = 0x00;
                break;

            case 5:
                pAppCtxt->desiredSpeed = CY_USBD_USB_DEV_SS_GEN2X2;
                break;

            default:
                pAppCtxt->desiredSpeed = CY_USBD_USB_DEV_SS_GEN1X2;
                break;
        }

        /* Select target driver on the host side. */
        if (pTestCfg->useWinUsb) {
            /* Send MSDR command to the handler. */
            glI2cSlaveRdBuffer[0] = 0x4D;
            glI2cSlaveRdBuffer[1] = 0x53;
            glI2cSlaveRdBuffer[2] = 0x44;
            glI2cSlaveRdBuffer[3] = 0x52;
            HandleI2cCommand(4UL);
        } else {
            /* Send CYDR command to the handler. */
            glI2cSlaveRdBuffer[0] = 0x43;
            glI2cSlaveRdBuffer[1] = 0x59;
            glI2cSlaveRdBuffer[2] = 0x44;
            glI2cSlaveRdBuffer[3] = 0x52;
            HandleI2cCommand(4UL);
        }

        /* Update the number of endpoints supported. */
        pAppCtxt->maxLbkPairs       = pTestCfg->numEps - 1;
        pAppCtxt->interruptEpEnable = ((pTestCfg->dmaConfig & 0x04) != 0);
        pAppCtxt->autoDmaEnable     = ((pTestCfg->dmaConfig & 0x02) != 0);
        pAppCtxt->burstModeEnable   = ((pTestCfg->dmaConfig & 0x01) != 0);

        /* Configure the endpoints as desired by the test configuration. */
        for (epIdx = 1; epIdx < pTestCfg->numEps; epIdx++) {
            glI2cSlaveRdBuffer[0]  = 0x45;
            glI2cSlaveRdBuffer[1]  = 0x50;
            glI2cSlaveRdBuffer[2]  = 0x53;
            glI2cSlaveRdBuffer[3]  = 0x4C;
            glI2cSlaveRdBuffer[4]  = epIdx;
            glI2cSlaveRdBuffer[5]  = epIdx;

            if ((pTestCfg->isoEpMask & (1UL << epIdx)) != 0) {
                glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_ISO;
                glI2cSlaveRdBuffer[7]  = pTestCfg->isoBufCount;
                glI2cSlaveRdBuffer[8]  = (uint8_t)(pTestCfg->isoBufSize & 0xFFU);
                glI2cSlaveRdBuffer[9]  = (uint8_t)(pTestCfg->isoBufSize >> 8U);
                glI2cSlaveRdBuffer[10] = (uint8_t)(pTestCfg->isoPktSize & 0xFFU);
                glI2cSlaveRdBuffer[11] = (uint8_t)(pTestCfg->isoPktSize >> 8U);
                if (pTestCfg->isoPktSize < 1024U) {
                    glI2cSlaveRdBuffer[12] = (uint8_t)(pTestCfg->isoPktSize & 0xFFU);
                    glI2cSlaveRdBuffer[13] = (uint8_t)(pTestCfg->isoPktSize >> 8U);
                } else {
                    glI2cSlaveRdBuffer[12] = 0x40;
                    glI2cSlaveRdBuffer[13] = 0x00;          /* Fixing FS max. pkt. size to 64 bytes. */
                }
                glI2cSlaveRdBuffer[14] = pTestCfg->isoPollRate;
                glI2cSlaveRdBuffer[15] = 0x00;          /* Unused */
                glI2cSlaveRdBuffer[16] = (uint8_t)(pTestCfg->isoPktSize & 0xFFU);
                glI2cSlaveRdBuffer[17] = (uint8_t)(pTestCfg->isoPktSize >> 8U);
                glI2cSlaveRdBuffer[18] = pTestCfg->isoMaxBurst;
                glI2cSlaveRdBuffer[19] = pTestCfg->isoBurstCnt;

                /* Use INTR endpoint instead of ISO where required. */
                if (pAppCtxt->interruptEpEnable) {
                    glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_INTR;
                    glI2cSlaveRdBuffer[19] = 0x00;
                }
            } else {
                glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_BULK;
                glI2cSlaveRdBuffer[7]  = pTestCfg->bulkBufCount;
                glI2cSlaveRdBuffer[8]  = (uint8_t)(pTestCfg->bulkBufSize & 0xFFU);
                glI2cSlaveRdBuffer[9]  = (uint8_t)(pTestCfg->bulkBufSize >> 8U);
                glI2cSlaveRdBuffer[10] = 0x00;
                glI2cSlaveRdBuffer[11] = 0x02;          /* Fixing HS max. pkt. size to 512 bytes. */
                glI2cSlaveRdBuffer[12] = 0x40;
                glI2cSlaveRdBuffer[13] = 0x00;          /* Fixing FS max. pkt. size to 64 bytes. */
                glI2cSlaveRdBuffer[14] = 0x00;          /* Polling rate is N/A for Bulk EP. */
                glI2cSlaveRdBuffer[15] = 0x00;          /* Unused */
                glI2cSlaveRdBuffer[16] = (uint8_t)(pTestCfg->bulkPktSize & 0xFFU);
                glI2cSlaveRdBuffer[17] = (uint8_t)(pTestCfg->bulkPktSize >> 8U);
                glI2cSlaveRdBuffer[18] = pTestCfg->bulkMaxBurst;
                glI2cSlaveRdBuffer[19] = 0x00;          /* MULT field is N/A for Bulk EP. */
            }

            HandleI2cCommand(20UL);
        }
    } else {

        lastEpIndex = pAppCtxt->maxLbkPairs;

        /* Use a default loop-back configuration for cases where control interface is not accessible. */
        for (epIdx = 1; epIdx <= lastEpIndex; epIdx++)
        {
            /* EP 1-OUT and 1-IN are configured as Sink+Source Endpoints for throughput measurement. */
            if (epIdx == 1)
            {
                glI2cSlaveRdBuffer[0]  = 0x53;
                glI2cSlaveRdBuffer[1]  = 0x53;
                glI2cSlaveRdBuffer[2]  = 0x45;
                glI2cSlaveRdBuffer[3]  = 0x50;
                glI2cSlaveRdBuffer[4]  = epIdx;
                glI2cSlaveRdBuffer[5]  = epIdx;
                glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_BULK;
                glI2cSlaveRdBuffer[7]  = 0x03;                  /* 3 DMA buffers. */
                glI2cSlaveRdBuffer[8]  = 0x00;
                glI2cSlaveRdBuffer[9]  = 0xC0;                  /* Each buffer is of size 48 KB. */
                glI2cSlaveRdBuffer[10] = 0x00;
                glI2cSlaveRdBuffer[11] = 0x02;                  /* HS max. pkt. size is 512 */
                glI2cSlaveRdBuffer[12] = 0x40;
                glI2cSlaveRdBuffer[13] = 0x00;                  /* FS max. pkt. size is 64 */
                glI2cSlaveRdBuffer[14] = 0x00;                  /* No polling for bulk EP */
                glI2cSlaveRdBuffer[15] = 0x00;                  /* Reserved */
                glI2cSlaveRdBuffer[16] = 0x00;
                glI2cSlaveRdBuffer[17] = 0x04;                  /* SS max. pkt. size is 1024 */
                glI2cSlaveRdBuffer[18] = 0x10;                  /* Burst of 16 packets for EP1. */
                glI2cSlaveRdBuffer[19] = 0x00;                  /* No streams support */

                HandleI2cCommand(20UL);
                continue;
            }

            /* Configure bulk loop back EP pair. */
            glI2cSlaveRdBuffer[0]  = 0x45;
            glI2cSlaveRdBuffer[1]  = 0x50;
            glI2cSlaveRdBuffer[2]  = 0x53;
            glI2cSlaveRdBuffer[3]  = 0x4C;

            glI2cSlaveRdBuffer[4]  = epIdx;
            glI2cSlaveRdBuffer[5]  = epIdx;

            switch (epIdx) {
#if (!USB3_LPM_ENABLE)
                /*
                 * When building with LPM enabled for compliance testing, enable only bulk endpoints.
                 * If INTR/ISO endpoints are required, they need to be added in different alternate
                 * settings which is not supported by this application.
                 */
                case 2:
                    /* Configure EP 2-OUT and 2-IN as Interrupt endpoints with a burst setting of 1 packet. */
                    glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_INTR;
                    glI2cSlaveRdBuffer[7]  = 0x02;              /* 2 buffers */
                    glI2cSlaveRdBuffer[8]  = 0x00;
                    glI2cSlaveRdBuffer[9]  = 0x04;              /* buffer size is 1 KB. */
                    glI2cSlaveRdBuffer[14] = 0x01;              /* Polling interval: Once per ITP. */
                    glI2cSlaveRdBuffer[18] = 0x01;              /* Burst of 1 packet */
                    glI2cSlaveRdBuffer[19] = 0x00;              /* Reserved. */
                    break;

                case 3:
                    /* Configure EP 3-OUT and 3-IN as Isochronous endpoints sending one 8 KB burst per interval. */
                    glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_ISO;
                    glI2cSlaveRdBuffer[7]  = 0x04;              /* 4 buffers */
                    glI2cSlaveRdBuffer[8]  = 0x00;
                    glI2cSlaveRdBuffer[9]  = 0x20;              /* buffer size is 8 KB. */
                    glI2cSlaveRdBuffer[14] = 0x01;              /* Polling interval: Once per ITP. */
                    glI2cSlaveRdBuffer[18] = 0x08;              /* Burst of 8 packets */
                    glI2cSlaveRdBuffer[19] = 0x00;              /* One burst per interval. */
                    break;
#endif /* (!USB3_LPM_ENABLE) */

                default:
                    glI2cSlaveRdBuffer[6]  = (uint8_t)CY_USB_ENDP_TYPE_BULK;
                    glI2cSlaveRdBuffer[7]  = 0x02;              /* 2 buffers */
                    glI2cSlaveRdBuffer[8]  = 0x00;
                    glI2cSlaveRdBuffer[9]  = 0x40;              /* buffer size is 16 KB. */
                    glI2cSlaveRdBuffer[14] = 0x00;              /* No polling for bulk EP */
                    glI2cSlaveRdBuffer[18] = 0x10;              /* Burst of 16 packets */
                    glI2cSlaveRdBuffer[19] = 0x00;              /* No streams support */
                    break;
            }


            glI2cSlaveRdBuffer[10] = 0x00;
            glI2cSlaveRdBuffer[11] = 0x02;                  /* HS max. pkt. size is 512 */
            glI2cSlaveRdBuffer[12] = 0x40;
            glI2cSlaveRdBuffer[13] = 0x00;                  /* FS max. pkt. size is 64 */
            glI2cSlaveRdBuffer[15] = 0x00;                  /* Unused */
            glI2cSlaveRdBuffer[16] = 0x00;
            glI2cSlaveRdBuffer[17] = 0x04;                  /* SS max. pkt. size is 1024 */

            HandleI2cCommand(20UL);
        }

#if USE_WINUSB
        /* Send MSDR command to the handler. */
        glI2cSlaveRdBuffer[0] = 0x4D;
        glI2cSlaveRdBuffer[1] = 0x53;
        glI2cSlaveRdBuffer[2] = 0x44;
        glI2cSlaveRdBuffer[3] = 0x52;
        HandleI2cCommand(4UL);
#else
        /* Send CYDR command to the handler. */
        glI2cSlaveRdBuffer[0] = 0x43;
        glI2cSlaveRdBuffer[1] = 0x59;
        glI2cSlaveRdBuffer[2] = 0x44;
        glI2cSlaveRdBuffer[3] = 0x52;
        HandleI2cCommand(4UL);
#endif /* USE_WINUSB */
    }

    /* Register descriptors with the USB Stack. */
    CyApp_RegisterUsbDescriptors(pAppCtxt, pAppCtxt->desiredSpeed);

    /* Send CONN command to the handler. */
    glI2cSlaveRdBuffer[0] = 0x43;
    glI2cSlaveRdBuffer[1] = 0x4F;
    glI2cSlaveRdBuffer[2] = 0x4E;
    glI2cSlaveRdBuffer[3] = 0x4E;
    HandleI2cCommand(4UL);

#else /* Use configuration provided through I2C interface. */

    DBG_APP_INFO("Waiting for I2C Cmd\r\n");
    while (pAppCtxt->usbConnectEnabled == false)
    {
        AppBlockingDelay(10);
    }

#endif /* USE_DFLT_CONFIG */
}

#if (FREERTOS_ENABLE)

#define APP_DMASET_GET_DMA_DESC(dmaset,n)       (cy_stc_dma_descriptor_t *)(&((dmaset)->dmaXferDscr[n]))
#define APP_DMASET_GET_DMAC_DESC(dmaset,n)      (cy_stc_dmac_descriptor_t *)(&((dmaset)->dmaXferDscr[n]))

/*
 * When device enters deep sleep and then wakes up, all the data that is currently sitting
 * in the Egress EPM is lost. We need to save and restore this data across deep sleep
 * state.
 *
 * For each of the IN endpoints, we check if there is an ongoing DMA request as well
 * as the status of the Egress SRAM. There are three possible cases:
 *
 * 1. Part of the last requested DMA transfer is already moved to Egress SRAM and the
 *    rest of DMA transfer is pending availability of space in the Egress SRAM. In this
 *    case, we calculate the RAM address from which data transfer needs to be resumed
 *    and the remaining data size.
 * 2. The DMA transfer has been completed, but a bit of the data is still sitting in
 *    the Egress SRAM. In this case, we store the RAM address from which the last
 *    data was copied so that it can be copied to the Egress SRAM again after deep
 *    sleep exit.
 * 3. Some part of the previous DMA transfer is sitting in the Egress SRAM and a new
 *    DMA transfer has already been queued. In this case, we have no way of locating
 *    the data that is currently in the Egress SRAM. Deep sleep entry is not allowed
 *    in this case.
 */

/** @brief Structure used to store information about the IN Endpoint DMA transfers
 *  that need to be restarted after waking from deep sleep.
 */
typedef struct {
    uint8_t *pDataBuf;                  /**< Pointer to memory containing the data to be copied into EPM. */
    uint16_t dataSize;                  /**< Size of data to be copied into EPM. */
} cy_stc_app_egress_ep_state_t;

cy_stc_app_egress_ep_state_t gl_EgressEpState[CY_USB_MAX_ENDP_NUMBER];

static bool PrepareForDeepSleep (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_app_endp_dma_set_t *pDmaSet;
    uint32_t epNumber;
    uint32_t dbgEp, t1;
    bool allowSleep = true;

    /* Block all further data transfer by setting NAKALL bit. */
    Cy_USB_USBD_EndpSetClearNakNrdyAll(pAppCtxt->pUsbdCtxt, true);

    for (epNumber = 1; epNumber < CY_USB_MAX_ENDP_NUMBER; epNumber++) {
        gl_EgressEpState[epNumber].pDataBuf = NULL;
        gl_EgressEpState[epNumber].dataSize = 0;

        pDmaSet = &(pAppCtxt->endpInDma[epNumber]);
        if (!(pDmaSet->valid))
            continue;

        dbgEp = MXS40USBHSDEV_USBHSDEV->EEPM_DEBUG_ENDPOINT[epNumber];

        if (
                (pAppCtxt->numLbkPairs <= 2)
           ) {
            /* Check whether we have an ongoing data transfer into the endpoint. */
            if (Cy_DMAC_Channel_IsEnabled(pCpuDmacBase, pDmaSet->channel)) {
                /* Calculate how much data is left in the DMA transfer. */
                if (Cy_DMAC_Channel_GetCurrentDescriptor(pCpuDmacBase, pDmaSet->channel) ==
                        APP_DMASET_GET_DMAC_DESC(pDmaSet, 0)) {
                    /* 2D descriptor not complete. Find out how many packets have been completed. */
                    t1 = Cy_DMAC_Channel_GetCurrentYloopIndex(pCpuDmacBase, pDmaSet->channel);

                    if ((t1 == 0) && ((dbgEp & 0x03) != 0)) {
                        /* There is no option to restore the data that is currently sitting in EPM.
                         * Block deep sleep entry.
                         */
                        DBG_APP_TRACE("EP %d busy: %x %x\r\n", epNumber, t1, dbgEp);
                        allowSleep = false;
                        break;
                    }

                    t1 -= (dbgEp & 0x03);
                } else {
                    if (pDmaSet->curDataSize <= pDmaSet->maxPktSize) {
                        if ((dbgEp & 0x03) != 0) {
                            DBG_APP_TRACE("EP %d busy: %x\r\n", epNumber, dbgEp);
                            allowSleep = false;
                            break;
                        } else {
                            t1 = 0;
                        }
                    } else {
                        t1 = Cy_DMAC_Descriptor_GetYloopDataCount(APP_DMASET_GET_DMAC_DESC(pDmaSet, 0));
                        t1 -= (dbgEp & 0x03);
                    }
                }

                gl_EgressEpState[epNumber].pDataBuf = pDmaSet->pCurDataBuffer + t1 * pDmaSet->maxPktSize;
                gl_EgressEpState[epNumber].dataSize = pDmaSet->curDataSize - t1 * pDmaSet->maxPktSize;
            } else {
                /* No ongoing transfer into the endpoint. Check if the EPM has data sitting there. */
                if ((dbgEp & 0x03) != 0) {
                    t1 = (pDmaSet->curDataSize + pDmaSet->maxPktSize - 1) / pDmaSet->maxPktSize;
                    t1 -= (dbgEp & 0x03);

                    gl_EgressEpState[epNumber].pDataBuf = pDmaSet->pCurDataBuffer + t1 * pDmaSet->maxPktSize;
                    gl_EgressEpState[epNumber].dataSize = pDmaSet->curDataSize - t1 * pDmaSet->maxPktSize;
                }
            }
        } else {
            /* Check whether we have an ongoing data transfer into the endpoint. */
            if (Cy_DMA_Channel_IsEnabled(pCpuDw1Base, epNumber)) {
                /* Calculate how much data is left in the DMA transfer. */
                if (Cy_DMA_Channel_GetCurrentDescriptor(pCpuDw1Base, epNumber) ==
                        APP_DMASET_GET_DMA_DESC(pDmaSet, 0)) {
                    /* 2D descriptor not complete. Find out how many packets have been completed. */
                    t1 = Cy_DMA_Channel_GetCurrentYIndex(pCpuDw1Base, epNumber);

                    if ((t1 == 0) && ((dbgEp & 0x03) != 0)) {
                        /* There is no option to restore the data that is currently sitting in EPM.
                         * Block deep sleep entry.
                         */
                        DBG_APP_TRACE("EP %d busy: %x %x\r\n", epNumber, t1, dbgEp);
                        allowSleep = false;
                        break;
                    }

                    t1 -= (dbgEp & 0x03);
                } else {
                    if (pDmaSet->curDataSize <= pDmaSet->maxPktSize) {
                        if ((dbgEp & 0x03) != 0) {
                            DBG_APP_TRACE("EP %d busy: %x\r\n", epNumber, dbgEp);
                            allowSleep = false;
                            break;
                        } else {
                            t1 = 0;
                        }
                    } else {
                        t1 = Cy_DMA_Descriptor_GetYloopDataCount(APP_DMASET_GET_DMA_DESC(pDmaSet, 0));
                        t1 -= (dbgEp & 0x03);
                    }
                }

                gl_EgressEpState[epNumber].pDataBuf = pDmaSet->pCurDataBuffer + t1 * pDmaSet->maxPktSize;
                gl_EgressEpState[epNumber].dataSize = pDmaSet->curDataSize - t1 * pDmaSet->maxPktSize;
            } else {
                /* No ongoing transfer into the endpoint. Check if the EPM has data sitting there. */
                if ((dbgEp & 0x03) != 0) {
                    t1 = (pDmaSet->curDataSize + pDmaSet->maxPktSize - 1) / pDmaSet->maxPktSize;
                    t1 -= (dbgEp & 0x03);

                    gl_EgressEpState[epNumber].pDataBuf = pDmaSet->pCurDataBuffer + t1 * pDmaSet->maxPktSize;
                    gl_EgressEpState[epNumber].dataSize = pDmaSet->curDataSize - t1 * pDmaSet->maxPktSize;
                }
            }
        }
    }

    if (allowSleep) {
        for (epNumber = 1; epNumber < CY_USB_MAX_ENDP_NUMBER; epNumber++) {
            /* Disable the DMA channels. */
            Cy_DMA_Channel_Disable(pCpuDw1Base, epNumber);
        }

        /* Flush the egress EPM. */
        Cy_USBD_FlushEndpAll(pAppCtxt->pUsbdCtxt, CY_USB_ENDP_DIR_IN);
    } else {
        /* Clear the NAKALL bit. */
        Cy_USB_USBD_EndpSetClearNakNrdyAll(pAppCtxt->pUsbdCtxt, false);
    }

    return allowSleep;
}

static void RestorePreSleepConfig (cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_app_endp_dma_set_t *pDmaSet;
    uint32_t epNumber;

    /* Trigger needs to be given for the next EP0-IN request. */
    pAppCtxt->pUsbdCtxt->ep0SendDone = false;

    for (epNumber = 1; epNumber < CY_USB_MAX_ENDP_NUMBER; epNumber++) {
        pDmaSet = &(pAppCtxt->endpInDma[epNumber]);
        if (!(pDmaSet->valid))
            continue;

        pDmaSet->firstRqtDone = false;

        if (gl_EgressEpState[epNumber].pDataBuf != NULL) {
            DBG_APP_INFO("Requeue: ep=%d size=%x\r\n", epNumber, gl_EgressEpState[epNumber].dataSize);
            Cy_USB_AppQueueWrite(pAppCtxt, epNumber, gl_EgressEpState[epNumber].pDataBuf,
                    gl_EgressEpState[epNumber].dataSize);

            gl_EgressEpState[epNumber].pDataBuf = NULL;
            gl_EgressEpState[epNumber].dataSize = 0;
        }
    }

    /* Clear the NAKALL bit after all state is restored. */
    Cy_USB_USBD_EndpSetClearNakNrdyAll(pAppCtxt->pUsbdCtxt, false);
}

#endif /* (FREERTOS_ENABLE) */

/*****************************************************************************
* Function Name: Cy_USBSS_DeInit
******************************************************************************
* Summary:
*  Temporary function to reset the USB block and GTX PHY to ensure device
*  disconnects from the host.
*
* Parameters:
*  cy_stc_usbss_cal_ctxt_t *pCalCtxt: Pointer to USB-SS CAL context.

* Return:
*  None
*****************************************************************************/
void Cy_USBSS_DeInit(cy_stc_usbss_cal_ctxt_t *pCalCtxt)
{
    USB32DEV_Type *base = pCalCtxt->regBase;
    USB32DEV_MAIN_Type  *USB32DEV_MAIN = &base->USB32DEV_MAIN;

    /* Disable the clock for USB3.2 function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_CLK_EN_Msk;

    /* Disable PHYSS */
    base->USB32DEV_PHYSS.USB40PHY[1].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    base->USB32DEV_PHYSS.USB40PHY[0].USB40PHY_TOP.TOP_CTRL_0 &=
                    ~(USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_RX_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PWR_GOOD_CORE_PLL_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_VBUS_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_REG_PHYSS_EN_Msk |
                     USB32DEV_PHYSS_USB40PHY_TOP_CTRL_0_PCLK_EN_Msk);

    /* Disable the SuperSpeed Device function */
    USB32DEV_MAIN->CTRL &= ~USB32DEV_MAIN_CTRL_SSDEV_ENABLE_Msk;
}

bool UsbSSConnectionEnable(void)
{

#if DEBUG_INFRA_EN
#if USBFS_LOGS_ENABLE
	vTaskDelay(1000);
#endif /* USBFS_LOGS_ENABLE */
#endif /* DEBUG_INFRA_EN */

    Cy_USBD_ConnectDevice(appCtxt.pUsbdCtxt, appCtxt.desiredSpeed);
    appCtxt.usbConnectDone = true;
    return true;
}

/*****************************************************************************
* Function Name: DisableUsbBlock
******************************************************************************
* Summary:
*  Function to disable the USB32DEV IP block after terminating current
*  connection.
*
* Parameters:
*  None

* Return:
*  None
*****************************************************************************/
static void DisableUsbBlock (void)
{
    /* Disable the USB32DEV IP. */
    USB32DEV->USB32DEV_MAIN.CTRL &= ~USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;

    /* Disable HBDMA adapter interrupts and the adapter itself. */
#if CY_CPU_CORTEX_M4
    NVIC_DisableIRQ(lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn);
    NVIC_DisableIRQ(lvds2usb32ss_usb32_egrs_dma_int_o_IRQn);
#else
    NVIC_DisableIRQ(NvicMux1_IRQn);
    NVIC_DisableIRQ(NvicMux2_IRQn);
#endif /* CY_CPU_CORTEX_M4 */

    Cy_HBDma_DeInit(&HBW_DrvCtxt);
    DBG_APP_INFO("DisableUSBBlock\r\n");
}

/*****************************************************************************
* Function Name: EnableUsbBlock
******************************************************************************
* Summary:
*  Function to enable the USB32DEV IP block before enabling a new USB
*  connection.
*
* Parameters:
*  None

* Return:
*  None
*****************************************************************************/
static void EnableUsbBlock (void)
{
    /* Enable the USB DMA adapters and respective interrupts. */
    Cy_HBDma_Init(NULL, USB32DEV, &HBW_DrvCtxt, 0, 0);

#if CY_CPU_CORTEX_M4
    NVIC_EnableIRQ(lvds2usb32ss_usb32_ingrs_dma_int_o_IRQn);
    NVIC_EnableIRQ(lvds2usb32ss_usb32_egrs_dma_int_o_IRQn);
#else
    NVIC_EnableIRQ(NvicMux1_IRQn);
    NVIC_EnableIRQ(NvicMux2_IRQn);
#endif /* CY_CPU_CORTEX_M4 */

    /* Make sure to enable USB32DEV IP first. */
    USB32DEV->USB32DEV_MAIN.CTRL |= USB32DEV_MAIN_CTRL_IP_ENABLED_Msk;
}

static uint16_t gCurUsbEvtLogIndex = 0;

/*****************************************************************************
* Function Name: AppPrintUsbEventLog
******************************************************************************
* Summary:
*  Function to print out the USB event log buffer content.
*
* Parameters:
*  pAppCtxt: Pointer to application context data structure.
*  pSSCal: Pointer to SSCAL context data structure.
*
* Return:
*  void
*****************************************************************************/
void AppPrintUsbEventLog (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usbss_cal_ctxt_t *pSSCal)
{
    uint16_t prevLogIdx = gCurUsbEvtLogIndex;

    (void)pSSCal;

    /* Print out any pending USB event log data. */
    gCurUsbEvtLogIndex = Cy_USBD_GetEvtLogIndex(pAppCtxt->pUsbdCtxt);
    while (gCurUsbEvtLogIndex != prevLogIdx) {
        DBG_APP_INFO("USBEVT: %x\r\n", pAppCtxt->pUsbEvtLogBuf[prevLogIdx]);
        prevLogIdx++;
        if (prevLogIdx == 512u) {
            prevLogIdx = 0u;
        }
    }
}

/*****************************************************************************
* Function Name: UsbDeviceTaskHandler
******************************************************************************
* Summary:
*  Entry function for the USB Data Loop-back task.
*
* Parameters:
*  pTaskParam: Application context pointer (opaque pointer).

* Return:
*  Does not return.
*****************************************************************************/
void
UsbDeviceTaskHandler (void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    uint8_t idx;
    bool intrEnabled = false;
    bool workPending;
    uint32_t lastPrintTime = 0, printCnt = 0;
#if FREERTOS_ENABLE
    EventBits_t evStat;
    TickType_t loopdelay = 50;
#endif /* FREERTOS_ENABLE */
    cy_en_usbss_lnk_power_mode_t curLinkState;
    uint32_t lpEntryTime = 0;
    uint32_t uartRxChar;

    (void)printCnt;

    DBG_APP_INFO("LoopBackTask started\r\n");

    /* Initialize application layer. */
    Cy_USB_AppInit(pAppCtxt, &usbdCtxt, pCpuDmacBase, pCpuDw0Base, pCpuDw1Base, &HBW_MgrCtxt);
    pAppCtxt->usbConnectEnabled     = false;
    pAppCtxt->usbConnectDone        = false;
    pAppCtxt->vbusPresent           = true;
    pAppCtxt->vbusChangeIntr        = false;
    pAppCtxt->reconnectTimeStamp    = 0;
    pAppCtxt->functionWakeTimestamp = 0;

    /* Register callback functions. */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    /* Update USB device configuration and wait until connect command is received. */
    GetUsbDeviceConfiguration(pAppCtxt);

    if (vBusDetDisable) {
        /* If VBus detection is disabled, assume that VBus is present. */
        pAppCtxt->vbusPresent = true;
    } else {
        /* Enable the VBus detect GPIO interrupt. */
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);

        /* Update the VBus presence status at start-up. */
        pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    }
    Cy_USB_AppSignalTask(&appCtxt, EV_DEVSTATE_CHG);

#if REMOTEWAKE_EN
    /* Configure the GPIO used to trigger remote wake. */
    ConfigureRemoteWakeTriggerPin();
#endif /* REMOTEWAKE_EN */

    while (1)
    {
#if FREERTOS_ENABLE
        /* Wait for some work to be queued. */
        evStat = xEventGroupWaitBits(pAppCtxt->appEvGrpHandle, TASK_WAIT_EV_MASK, pdTRUE, pdFALSE, loopdelay);

#else
        Cy_Debug_PrintLog();
#endif /* FREERTOS_ENABLE */

        if ((pAppCtxt->usbConnectEnabled == false) && (pAppCtxt->reconnectTimeStamp != 0) &&
                (Cy_USBD_GetTimerTick() >= pAppCtxt->reconnectTimeStamp)) {
            pAppCtxt->reconnectTimeStamp = 0;
            pAppCtxt->usbConnectEnabled  = true;
            Cy_USB_AppSignalTask(pAppCtxt, EV_DEVSTATE_CHG);
        }

        while (Cy_SCB_UART_GetNumInRxFifo(LOGGING_SCB) != 0) {
            uartRxChar = Cy_SCB_ReadRxFifo(LOGGING_SCB);
            (void)uartRxChar;
#if REMOTEWAKE_EN
            if ((char)uartRxChar == 'W') {
                InitiateRemoteWake();
            }
#endif /* REMOTEWAKE_EN */
        }

        /* Print a status message and the event log buffer content once in 10 seconds. */
        if ((Cy_USBD_GetTimerTick() - lastPrintTime) >= 10000UL) {
            lastPrintTime = Cy_USBD_GetTimerTick();

            DBG_APP_INFO("TASKLOOP: %d (%x)\r\n", printCnt++, USB32DEV->USB32DEV_LNK.LNK_DEVICE_POWER_CONTROL);

            /* Print out any pending USB event log data. */
            AppPrintUsbEventLog(pAppCtxt, &ssCalCtxt);

            /* Print the CTLE RX adaptation results if the algorithm has been run since last check. */
            Cy_USBSS_Cal_PrintCtleResults(&ssCalCtxt, 0);
            Cy_USBSS_Cal_PrintCtleResults(&ssCalCtxt, 1);

#if REMOTEWAKE_EN
            gl_RemoteWakeDone = false;
#endif /* REMOTEWAKE_EN */
        }

#if USB3_LPM_ENABLE
        if ((pAppCtxt->isLpmEnabled == false) && (pAppCtxt->lpmEnableTime != 0)) {
            Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);

            if (
                    (Cy_USBD_GetTimerTick() >= pAppCtxt->lpmEnableTime) &&
                    (curLinkState != CY_USBSS_LPM_U3)
               ) {
                pAppCtxt->isLpmEnabled  = true;
                pAppCtxt->lpmEnableTime = 0;
                Cy_USBD_LpmEnable(pAppCtxt->pUsbdCtxt);
            }
        }
#endif /* USB3_LPM_ENABLE */

#if FREERTOS_ENABLE
        if ((evStat & EV_DEVSTATE_CHG) != 0)
#endif /* FREERTOS_ENABLE */
        {
            if (pAppCtxt->vbusChangeIntr) {
                /*
                 * Debounce delay of 370 ms when VBus turns from ON to OFF and 55 ms when VBus turns from
                 * OFF to ON.
                 */
                if (pAppCtxt->vbusPresent) {
                    AppBlockingDelay(370);
                } else {
                    AppBlockingDelay(55);
                }

                /* Clear the VBus change flag, clear and re-enable the interrupt. */
                pAppCtxt->vbusChangeIntr = false;
                Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
                Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);

                if (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE) {
                    if (!pAppCtxt->vbusPresent) {
                        Cy_USBD_AddEvtToLog(pAppCtxt->pUsbdCtxt, CY_SSCAL_EVT_VBUS_PRESENT);
                        DBG_APP_INFO("VBus presence detected\r\n");
                        pAppCtxt->vbusPresent = true;
                    } else {
                        DBG_APP_TRACE("Spurious GPIO INT - 1\r\n");
                    }
                } else {
                    if (pAppCtxt->vbusPresent) {
                        Cy_USBD_AddEvtToLog(pAppCtxt->pUsbdCtxt, CY_SSCAL_EVT_VBUS_ABSENT);
                        DBG_APP_INFO("VBus absence detected\r\n");
                        pAppCtxt->vbusPresent = false;
                    } else {
                        DBG_APP_TRACE("Spurious GPIO INT - 0\r\n");
                    }
                }
            }

            if (pAppCtxt->usbConnectEnabled == true) {
                if ((pAppCtxt->usbConnectDone == true) &&
                        ((pAppCtxt->vbusPresent == false) || (pAppCtxt->ccDisconnectDetect == true))) {

                    Cy_USB_AppDisableEndpDma(pAppCtxt);
                    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
                    Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
                    DBG_APP_INFO("Disconnect due to VBus loss\r\n");
                    pAppCtxt->usbConnectDone = false;
                    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
                    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;

                    DisableUsbBlock();
                }

                if ((pAppCtxt->vbusPresent == true) && (pAppCtxt->ccDisconnectDetect == false) &&
                        (pAppCtxt->usbConnectDone == false)) {
                    DBG_APP_INFO("Connect due to VBus presence\r\n");
                    pAppCtxt->reconnectTimeStamp    = 0;
                    pAppCtxt->functionWakeTimestamp = 0;
                    EnableUsbBlock();

                    /* Prevent application level prints for some time while USB connection is being attempted. */
                    lastPrintTime = Cy_USBD_GetTimerTick();

                    if (!UsbSSConnectionEnable())
                    {
                        Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
                        Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
                        pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
                        Cy_USB_AppSignalTask(&appCtxt, EV_DEVSTATE_CHG);
                    }
                }
            } else {
                if (pAppCtxt->usbConnectDone) {
                    Cy_USB_AppDisableEndpDma(pAppCtxt);
                    Cy_USBD_DisconnectDevice(pAppCtxt->pUsbdCtxt);
                    Cy_USBSS_DeInit(pAppCtxt->pUsbdCtxt->pSsCalCtxt);
                    DBG_APP_INFO("Disconnect on user request\r\n");
                    pAppCtxt->usbConnectDone = false;
                    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
                    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;

                    DisableUsbBlock();
                }
            }
        }

#if FREERTOS_ENABLE
        if ((evStat & EV_VENDOR_REQUEST) != 0) {
            Cy_USB_AppVendorRqtHandler(pAppCtxt);
        }
#else
        if (VendorCtrlRqtPending) {
            VendorCtrlRqtPending = false;
            Cy_USB_AppVendorRqtHandler(pAppCtxt);
        }
#endif /* FREERTOS_ENABLE */

        /*
         * If we are/were in the configured state, we can run the loopback task.
         * Otherwise, cancel all DMA handling.
         */
        if (
                (pAppCtxt->usbConnectDone == false) ||
                (
                 (pAppCtxt->devState != CY_USB_DEVICE_STATE_CONFIGURED) &&
                 (pAppCtxt->prevDevState != CY_USB_DEVICE_STATE_CONFIGURED)
                )
           ) {
            if (intrEnabled) {
                ResetLoopBackCtxt(pAppCtxt);
                intrEnabled = false;
            }
        } else {
            if (!intrEnabled) {
                /* Register DMA transfer completion interrupts. */
                EnableEpDmaISRs(pAppCtxt);
                intrEnabled = true;
                lpEntryTime = Cy_USBD_GetTimerTick();
            }

            curLinkState = CY_USBSS_LPM_UNKNOWN;
            if ((pAppCtxt->isAppActive) && (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1)) {
                Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);
            }

#if FREERTOS_ENABLE
            if (glDeepSleepEnabled) {
                uint32_t dpslpcnt = 500;
                uint32_t tmpState;

                if (pAppCtxt->devSpeed >= CY_USBD_USB_DEV_SS_GEN1) {
                    if (curLinkState == CY_USBSS_LPM_U3) {
                        do {
                            Cy_SysLib_Delay(1);
                            Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);
                            dpslpcnt--;
                        } while ((curLinkState == CY_USBSS_LPM_U3) && (dpslpcnt > 0));

                        if ((dpslpcnt == 0) && (curLinkState == CY_USBSS_LPM_U3)) {
                            if (Cy_USBSS_Cal_DeepSleepPrep(&ssCalCtxt)) {

                                AppPrintUsbEventLog(pAppCtxt, &ssCalCtxt);
                                Cy_USBSS_Cal_PrintCtleResults(&ssCalCtxt, 0);
                                Cy_USBSS_Cal_PrintCtleResults(&ssCalCtxt, 1);

                                DBG_APP_INFO("Entering deepsleep\r\n");
                                Cy_SysLib_DelayUs(1500);

                                tmpState = Cy_SysLib_EnterCriticalSection();
                                Cy_USBSS_Cal_GetLinkPowerState(pAppCtxt->pUsbdCtxt->pSsCalCtxt, &curLinkState);
                                if (curLinkState == CY_USBSS_LPM_U3) {
                                    Cy_USBHS_Cal_DeinitPLL(&hsCalCtxt);
                                    Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
                                    Cy_USBHS_Cal_InitPLL(&hsCalCtxt);
                                }
                                Cy_SysLib_ExitCriticalSection(tmpState);
                            }
                        }
                    }
                } else {
                    if (
                            (pAppCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) &&
                            (pAppCtxt->prevDevState == CY_USB_DEVICE_STATE_CONFIGURED)
                       ) {
                        do {
                            Cy_SysLib_Delay(1);
                            dpslpcnt--;
                        } while ((pAppCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) && (dpslpcnt > 0));

                        if ((pAppCtxt->devState == CY_USB_DEVICE_STATE_SUSPEND) && (dpslpcnt == 0)) {
                            /* Print message and delay to ensure UART TX FIFO gets flushed. */
                            AppPrintUsbEventLog(pAppCtxt, &ssCalCtxt);
                            DBG_APP_INFO("Entering deepsleep\r\n");
                            Cy_SysLib_DelayUs(1500);

                            if (PrepareForDeepSleep(pAppCtxt)) {
                                tmpState = Cy_SysLib_EnterCriticalSection();
                                Cy_USBHS_Cal_DeinitPLL(&hsCalCtxt);
                                Cy_SysPm_CpuEnterDeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
                                Cy_USBHS_Cal_InitPLL(&hsCalCtxt);
                                Cy_SysLib_ExitCriticalSection(tmpState);
                                RestorePreSleepConfig(pAppCtxt);
                            }
                        }
                    }
                }
            }
#endif /* FREERTOS_ENABLE */

            /*
             * If the link has been in USB2-L1 or in USB3-U2 for more than 0.5 second, initiate LPM exit so that
             * transfers do not get delayed significantly.
             */
            if (
                    (
                     ((curLinkState == CY_USBSS_LPM_U2) || (curLinkState == CY_USBSS_LPM_U1)) &&
                     (!(pAppCtxt->pUsbdCtxt->pSsCalCtxt->forceLPMAccept))
                    ) ||
                    (
                     (pAppCtxt->devSpeed <= CY_USBD_USB_DEV_HS) &&
                     ((MXS40USBHSDEV_USBHSDEV->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0)
                    )
               ) {
                if ((Cy_USBD_GetTimerTick() - lpEntryTime) > 500UL) {
                    lpEntryTime = Cy_USBD_GetTimerTick();
                    Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
                }
            } else {
                lpEntryTime = Cy_USBD_GetTimerTick();
            }

            /* Handle the data loopback tasks for all EP pairs. */
            for (idx = 0; idx < pAppCtxt->numLbkPairs; idx++) {
#if (!FREERTOS_ENABLE)
                if ((EpPairWorkPending & (1U << idx)) == 0) continue;
#endif /* FREERTOS_ENABLE */
                workPending = false;

                do {
                    workPending = HandleLoopBackTask(pAppCtxt, idx);
                } while (workPending);
            }
        }
    }
}

#if FREERTOS_ENABLE
/*****************************************************************************
* Function Name: DebugLogTaskHandler
******************************************************************************
* Summary:
*  Task that pushes debug log messages to the target interface.
*
* Parameters:
*  pTaskParam: Application context pointer (opaque pointer).

* Return:
*  Does not return.
*****************************************************************************/
void
DebugLogTaskHandler (void *pTaskParam)
{
    while (1)
    {
        Cy_Debug_PrintLog();
        vTaskDelay(5);
    }
}
#endif /* FREERTOS_ENABLE */

/* Const array that maps DMA clock selection parameter to the actual clock selection. */
const cy_en_hbdma_clk_freq_t DdfToDmaClkSel[8] = {
    CY_HBDMA_CLK_240_MHZ,
    CY_HBDMA_CLK_160_MHZ,
    CY_HBDMA_CLK_160_MHZ,
    CY_HBDMA_CLK_150_MHZ,
    CY_HBDMA_CLK_SSPHY_CLK,
    CY_HBDMA_CLK_SSPHY_CLK,
    CY_HBDMA_CLK_240_MHZ,
    CY_HBDMA_CLK_240_MHZ
};

/*****************************************************************************
* Function Name: main(void)
******************************************************************************
* Summary:
*  Entry to the application.
*
* Parameters:
*  void

* Return:
*  Does not return.
*****************************************************************************/
int main(void)
{
#if FREERTOS_ENABLE
    BaseType_t status;
#endif /* FREERTOS_ENABLE */

    cy_stc_testdev_config_t *pTestCfg = (cy_stc_testdev_config_t *)TESTDEV_CFG_STRUCT_ADDRESS;

    if ((pTestCfg->startSig == TESTDEV_CFG_START_SIG) && (pTestCfg->endSig == TESTDEV_CFG_END_SIG)) {
        if ((pTestCfg->clkConfig & 0x01U) != 0) {
            hfclkFreq = 50000000UL;
        } else {
            hfclkFreq = 75000000UL;
        }

        dmaClkSel = ((pTestCfg->clkConfig >> 1U) & 0x07U);

        if (pTestCfg->vBusDetEnabled & 0x01) {
            vBusDetDisable = false;
        } else {
            vBusDetDisable = true;
        }

        if (pTestCfg->vBusDetEnabled & 0x02) {
            glDeepSleepEnabled = true;
        } else {
            glDeepSleepEnabled = false;
        }
    } else {
        /* AXI clock frequency set to 240 MHz derived from USBHS PLL. */
        dmaClkSel = 0;

#if DEEPSLEEP_ENABLE
        glDeepSleepEnabled = true;
#else
        glDeepSleepEnabled = false;
#endif /* DEEPSLEEP_ENABLE */
    }

    /* Initialize the PDL driver library and set the clock variables. */
    Cy_PDL_Init(&cy_deviceIpBlockCfgFX3G2);
    cybsp_init();

    /* Get the CPU running frequency and store it. */
    hfclkFreq = Cy_SysClk_ClkFastGetFrequency();

    /* Unlock and then disable the watchdog. */
    Cy_WDT_Unlock();
    Cy_WDT_Disable();

#if CY_CPU_CORTEX_M4
    /*
     * If logging is done through the USBFS port, ISR execution is required in this application.
     * Set BASEPRI value to 0 to ensure all exceptions can run and then enable interrupts.
     */
    __set_BASEPRI(0);
#endif /* CY_CPU_CORTEX_M4 */
    __enable_irq ();

    memset((uint8_t *)&appCtxt, 0, sizeof(appCtxt));
    memset((uint8_t *)&ssCalCtxt, 0, sizeof(ssCalCtxt));
    memset((uint8_t *)&hsCalCtxt, 0, sizeof(hsCalCtxt));
    memset((uint8_t *)&usbdCtxt, 0, sizeof(usbdCtxt));

    pCpuDmacBase = ((DMAC_Type *)DMAC_BASE);
    pCpuDw0Base  = ((DW_Type *)DW0_BASE);
    pCpuDw1Base  = ((DW_Type *)DW1_BASE);

    UsbDevInit();

    /* Store IP base address in CAL context. */
    hsCalCtxt.pCalBase = MXS40USBHSDEV_USBHSDEV;
    hsCalCtxt.pPhyBase = MXS40USBHSDEV_USBHSPHY;
    ssCalCtxt.regBase  = USB32DEV;

    /*
     * Make sure any previous USB connection state is cleared. Give some delay to allow the host to process
     * disconnection.
     */
    Cy_USBSS_DeInit(&ssCalCtxt);
    Cy_SysLib_Delay(500);

    /* Initialize loopback context variables. */
    appCtxt.maxLbkPairs       = MAX_LP_PAIRS;
    appCtxt.numLbkPairs       = 0;
    appCtxt.burstModeEnable   = false;
    appCtxt.interruptEpEnable = false;

#if IP_IP_CHANNEL_AUTO
    appCtxt.autoDmaEnable = true;
#else
    appCtxt.autoDmaEnable = false;
#endif /* IP_IP_CHANNEL_AUTO */

    /* Initialize the HbDma IP and DMA Manager */
    InitHbDma();

    /* Initialize the USBD layer */
    Cy_USB_USBD_Init(&appCtxt, &usbdCtxt, pCpuDmacBase, &hsCalCtxt, &ssCalCtxt, &HBW_MgrCtxt);

    /* Retry link start-up up to 2 more times (total of 3 times) in case of training timeout. */
    Cy_USBD_HandleRxFailure(&usbdCtxt, true, 2);

    /* Set the desired DMA clock frequency. */
    Cy_USBD_SetDmaClkFreq(&usbdCtxt, DdfToDmaClkSel[dmaClkSel]);

    /*
     * Routing of debug pins for monitoring:
     * DDFT0          (P11.0): EGRESS_SCK_ACTV[1]
     * DDFT1          (P11.1): INGRESS_SCK_ACTV[1]
     * USB.GPIO_DDFT0 (P9.2) : TX_AFE_PULLUP
     * USB.GPIO_DDFT1 (P9.3) : PIPE_RX_ELECIDLE
     * SIP.GPIO_DDFT0 (P9.4) : Not used
     * SIP.GPIO_DDFT1 (P9.5) : Not used
     */
    Cy_UsbFx_SelectDFTFunctions(CY_FX_DBG_FUNC_USBSS_EG_SCK1_ACTIVE, CY_FX_DBG_FUNC_USBSS_IN_SCK1_ACTIVE,
            CY_FX_DBG_FUNC_USBPHY0_TX_PULLUP_EN, CY_FX_DBG_FUNC_USBPHY0_PIPE_RX_ELEC_IDLE,
            CY_FX_DBG_FUNC_NONE, CY_FX_DBG_FUNC_NONE);

    /* Allocate a memory block and register it as USB CAL event log buffer. */
    appCtxt.pUsbEvtLogBuf = (uint32_t *)g_UsbEvtLogBuf;
    Cy_USBD_InitEventLog(&usbdCtxt, appCtxt.pUsbEvtLogBuf, 512u);

#if FREERTOS_ENABLE
    status = xTaskCreate(DebugLogTaskHandler, "LoggerTask", 500, (void *)&appCtxt, 4, &(appCtxt.logTaskHandle));
    if (status != pdPASS) {
        while (1) {
        }
    }

    appCtxt.appEvGrpHandle = xEventGroupCreate();
    if (appCtxt.appEvGrpHandle != NULL)
    {
        status = xTaskCreate(UsbDeviceTaskHandler, "UsbAppTask", 1000, (void *)&appCtxt, 5, &(appCtxt.appTaskHandle));
        if (status != pdPASS)
        {
            DBG_APP_ERR("xTaskCreate failed\r\n");
        }
    }
    else
    {
        DBG_APP_ERR("xEventGroupCreate failed\r\n");
        status = pdFALSE;
    }

    if (status == pdPASS)
    {
        /* Start the RTOS kernel scheduler. */
        vTaskStartScheduler();
    }

    while (1)
    {
        Cy_SysLib_Delay(100);
    }
#else
    /* Start the SysTick timer. */
    SetupTimerInterrupt();

    /* Enable USB connection and start the data handling task. */
    UsbDeviceTaskHandler((void *)&appCtxt);
#endif /* FREERTOS_ENABLE */

    return 0;
}


/*****************************************************************************
 * Function Name: Cy_OnResetUser(void)
 ******************************************************************************
 * Summary:
 *  Init function which is executed before the load regions in RAM are updated.
 *  The High BandWidth subsystem needs to be enable here to allow variables
 *  placed in the High BandWidth SRAM to be updated.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *****************************************************************************/
void Cy_OnResetUser (void)
{
    Cy_UsbFx_OnResetInit();
}

/* [] END OF FILE */
