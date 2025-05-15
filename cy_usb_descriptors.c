/***************************************************************************//**
* \file cy_usb_descriptors.c
* \version 1.0
*
* Provides the USB descriptors and descriptor manipulation code used in the
* FX20 USB test application.
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
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"

/* Standard device descriptor for USB 2.0 */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB20DeviceDscr[] =
{
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x10,0x02,                      /* USB 2.10 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0xB4,0x04,                      /* Vendor ID */
    0xF1,0x00,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard device descriptor for USB 3.0 */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB30DeviceDscr[] =
{
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x20,0x03,                      /* USB 3.2 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 : 2^9 */
    0xB4,0x04,                      /* Vendor ID */
    0xF1,0x00,                      /* Product ID */
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x00,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard full speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFSConfigDscr[MAX_HS_CFG_DSCR_SIZE] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x12,0x00,                      /* Length of this descriptor and all sub descriptors. */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* COnfiguration string index */
#if REMOTEWAKE_EN
    0xA0,                           /* Config characteristics - bus powered with remote wake support. */
#else
    0x80,                           /* Config characteristics - bus powered without remote wake support. */
#endif /* REMOTEWAKE_EN */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of endpoints */
    0xFF,                           /* Interface class */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */
};

/* Standard high speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBHSConfigDscr[MAX_HS_CFG_DSCR_SIZE] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x12,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
#if REMOTEWAKE_EN
    0xA0,                           /* Config characteristics - bus powered with remote wake support. */
#else
    0x80,                           /* Config characteristics - bus powered without remote wake support. */
#endif /* REMOTEWAKE_EN */
    0x32,                           /* Max power consumption of device (in 2mA unit) : 100mA */

    /* Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of endpoints */
    0xFF,                           /* Interface class */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */
};

/* Standard super speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBSSConfigDscr[MAX_SS_CFG_DSCR_SIZE] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    0x02,                           /* Configuration descriptor type */
    0x12,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x01,                           /* Number of interfaces */
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
#if REMOTEWAKE_EN
    0xA0,                           /* Config characteristics - bus powered with remote wake support. */
#else
    0x80,                           /* Config characteristics - bus powered without remote wake support. */
#endif /* REMOTEWAKE_EN */
    0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */

    /* Interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface Descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points */
    0xFF,                           /* Interface class */
    0x00,                           /* Interface sub class */
    0x00,                           /* Interface protocol code */
    0x00                            /* Interface descriptor string index */
};

/* Device qualifier descriptor. */
USB3_DESC_ATTRIBUTES uint8_t CyFxDevQualDscr[] =
{
    0x0A,                           /* Descriptor size */
    0x06,                           /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0x00,                           /* Device class */
    0x00,                           /* Device sub-class */
    0x00,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Binary Object Store (BOS) Descriptor to be used in USB 2.x connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxBOSDscr_HS[] =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x0C,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x01,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */
};

/* Binary Object Store (BOS) Descriptor to be used in SuperSpeed connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxBOSDscr_Gen1[] =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x16,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: Not LTM capable.  */
    0x0E,0x00,                      /* Speeds supported by the device : SS Gen1, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07                       /* U2 Device Exit latency */
};

/* Binary Object Store (BOS) Descriptor to be used in SuperSpeedPlus connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxBOSDscr_Gen2[] =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x2D,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x04,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: Not LTM capable.  */
    0x0E,0x00,                      /* Speeds supported by the device : SS Gen1, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07,                      /* U2 Device Exit latency */

    /* SuperSpeedPlus USB device capability */
    0x14,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0A,                           /* SuperSpeedPlus Device capability */
    0x00,                           /* Reserved */
    0x01,0x00,0x00,0x00,            /* SSAC=1, SSIC=0 */
    0x00,0x11,                      /* SSID=0, Min. RX Lane = 1, Min. Tx Lane = 1 */
    0x00,0x00,                      /* Reserved */
    0x30,0x40,0x0A,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Rx), LP=1(SSPlus), LSM=10 */
    0xB0,0x40,0x0A,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Tx), LP=1(SSPlus), LSM=10 */

    /* Precision Time Measurement capability */
    0x03,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0B                            /* Precision Time Measurement (PTM) Capability Descriptor */
};

/* Binary Object Store (BOS) Descriptor to be used in SuperSpeedPlus connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxBOSDscr_Gen1x2[] =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x2A,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x03,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: Not LTM capable.  */
    0x0E,0x00,                      /* Speeds supported by the device : SS Gen1, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07,                      /* U2 Device Exit latency */

    /* SuperSpeedPlus USB device capability for G1X2 */
    0x14,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0A,                           /* SuperSpeedPlus Device capability */
    0x00,                           /* Reserved */
    0x01,0x00,0x00,0x00,            /* SSAC=1, SSIC=0 */
    0x00,0x22,                      /* SSID=0, Min. RX Lane = 2, Min. Tx Lane = 2 */
    0x00,0x00,                      /* Reserved */
    0x30,0x00,0x05,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Rx), LP=0(SS), LSM=5 */
    0xB0,0x00,0x05,0x00             /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Tx), LP=0(SS), LSM=5 */
};

/* Binary Object Store (BOS) Descriptor to be used in SuperSpeedPlus connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxBOSDscr_Gen2x2[] =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x2D,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x04,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: Not LTM capable.  */
    0x0E,0x00,                      /* Speeds supported by the device : SS Gen1, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07,                      /* U2 Device Exit latency */

    /* SuperSpeedPlus USB device capability for G2X2 */
    0x14,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0A,                           /* SuperSpeedPlus Device capability */
    0x00,                           /* Reserved */
    0x01,0x00,0x00,0x00,            /* SSAC=1, SSIC=0 */
    0x00,0x22,                      /* SSID=0, Min. RX Lane = 2, Min. Tx Lane = 2 */
    0x00,0x00,                      /* Reserved */
    0x30,0x40,0x0A,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Rx), LP=1(SSPlus), LSM=10 */
    0xB0,0x40,0x0A,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Tx), LP=1(SSPlus), LSM=10 */

    /* Precision Time Measurement capability */
    0x03,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0B                            /* Precision Time Measurement (PTM) Capability Descriptor */
};

USB3_DESC_ATTRIBUTES uint8_t CyFxLangString[] =
{
    0x04,
    0x03,
    0x09,
    0x04
};

USB3_DESC_ATTRIBUTES uint8_t CyFxMfgString[] =
{
    0x08,
    0x03,
    'I',
    0x00,
    'F',
    0x00,
    'X',
    0x00
};

USB3_DESC_ATTRIBUTES uint8_t CyFxProdString[] =
{
    0x0C,
    0x03,
    'F',
    0x00,
    'X',
    0x00,
    '3',
    0x00,
    'G',
    0x00,
    '2',
    0x00
};

/* HID Report Descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBReportDscr[] =
{
    0x05, 0x01,                         /* Usage Page (Generic Desktop) */
    0x09, 0x02,                         /* Usage (Mouse) */
    0xA1, 0x01,                         /* Collection (Application) */
    0x09, 0x01,                         /* Usage (Pointer) */
    0xA1, 0x00,                         /* Collection (Physical) */
    0x05, 0x01,                         /* Usage Page (Generic Desktop) */
    0x09, 0x30,                         /* Usage (X) */
    0x09, 0x31,                         /* Usage (Y) */
    0x15, 0x81,                         /* Logical Minimum (-127) */
    0x25, 0x7F,                         /* Logical Maximum (127) */
    0x75, 0x08,                         /* Report Size (8) */
    0x95, 0x02,                         /* Report Count (2) */
    0x81, 0x06,                         /* Input (Data, Value, Relative, Bit Field) */
    0xC0,                               /* End Collection */
    0xC0                                /* End Collection */
};

/* MS OS String Descriptor */
USB3_DESC_ATTRIBUTES uint8_t glOsString[] =
{
    0x12, /* Length. */
    0x03, /* Type - string. */
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00, '1', 0x00, '0', 0x00, '0', 0x00, /* Signature. */
    MS_VENDOR_CODE, /* MS vendor code. */
    0x00 /* Padding. */
};

USB3_DESC_ATTRIBUTES uint8_t glOsCompatibilityId[] =
{
    /* Header */
    0x28, 0x00, 0x00, 0x00, /* length Need to be updated based on number of interfaces. */
    0x00, 0x01, /* BCD version */
    0x04, 0x00, /* Index: 4 - compatibility ID */
    0x01, /* count. Need to be updated based on number of interfaces. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved. */
    /* First Interface */
    0x00, /* Interface number */
    0x01, /* reserved: Need to be 1. */
    0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, /* comp ID –ID to bind the device with
                                                       WinUSB.*/
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* sub-compatibility ID - NONE. */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* reserved - needs to be zero. */
};

USB3_DESC_ATTRIBUTES uint8_t glOsFeature[] =
{
    /* Header */
#if REMOTEWAKE_EN
    0xAA, 0x01, 0x00, 0x00, /* Total length. */
#else
    0x8E, 0x00, 0x00, 0x00, /* Total length. */
#endif /* REMOTEWAKE_EN */

    0x00, 0x01, /* BCD version. 1.0 as per MS */
    0x05, 0x00, /* Index: Should be set to 5 for OS feature descriptor. */

#if REMOTEWAKE_EN
    0x06, 0x00, /* count. */
#else
    0x01, 0x00, /* count. */
#endif /* REMOTEWAKE_EN */

#if REMOTEWAKE_EN
    /* Property section to enable Selective Suspend. */
    0x36, 0x00, 0x00, 0x00, /* length */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x24, 0x00,             /* wPropertyNameLength: (17 + 1) * 2 = 36 */
    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x64, 0x00,
    0x6c, 0x00, 0x65, 0x00, 0x45, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x62, 0x00, 0x6c, 0x00, 0x65, 0x00,
    0x64, 0x00, 0x00, 0x00, /* bPropertyName: DeviceIdleEnabled */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4 */
    0x01, 0x00, 0x00, 0x00, /* bPropertyData: 0x00000001 */

    /* Property section to enable user control of Selective Suspend. */
    0x44, 0x00, 0x00, 0x00, /* length */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x32, 0x00,             /* wPropertyNameLength: (24 + 1) * 2 = 50 */
    0x55, 0x00, 0x73, 0x00, 0x65, 0x00, 0x72, 0x00, 0x53, 0x00, 0x65, 0x00, 0x74, 0x00,
    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x64, 0x00,
    0x6c, 0x00, 0x65, 0x00, 0x45, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x62, 0x00, 0x6c, 0x00, 0x65, 0x00,
    0x64, 0x00, 0x00, 0x00, /* bPropertyName: UserSetDeviceIdleEnabled */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4 */
    0x01, 0x00, 0x00, 0x00, /* bPropertyData: 0x00000001 */

    /* Property section to set the default Idle state. */
    0x34, 0x00, 0x00, 0x00, /* length */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x22, 0x00,             /* wPropertyNameLength: (16 + 1) * 2 = 34 */
    0x44, 0x00, 0x65, 0x00, 0x66, 0x00, 0x61, 0x00, 0x75, 0x00, 0x6c, 0x00, 0x74, 0x00, 0x49, 0x00,
    0x64, 0x00, 0x6c, 0x00, 0x65, 0x00, 0x53, 0x00, 0x74, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00,
    0x00, 0x00,             /* bPropertyName: DefaultIdleState */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4 */
    0x01, 0x00, 0x00, 0x00, /* bPropertyData: 0x00000001 */

    /* Property section to set the default Idle timeout. */
    0x38, 0x00, 0x00, 0x00, /* length */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x26, 0x00,             /* wPropertyNameLength: (18 + 1) * 2 = 38 */
    0x44, 0x00, 0x65, 0x00, 0x66, 0x00, 0x61, 0x00, 0x75, 0x00, 0x6c, 0x00, 0x74, 0x00, 0x49, 0x00,
    0x64, 0x00, 0x6c, 0x00, 0x65, 0x00, 0x54, 0x00, 0x69, 0x00, 0x6d, 0x00, 0x65, 0x00, 0x6f, 0x00,
    0x75, 0x00, 0x74, 0x00, 0x00, 0x00, /* bPropertyName: DefaultIdleTimeout */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4 */
    0x88, 0x13, 0x00, 0x00, /* bPropertyData: 5000 = 0x00001388 */

    /* Property section to enable RemoteWake */
    0x36, 0x00, 0x00, 0x00, /* length */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_DWORD_LITTLE_ENDIAN */
    0x24, 0x00,             /* wPropertyNameLength: (17 + 1) * 2 = 36 */
    0x53, 0x00, 0x79, 0x00, 0x73, 0x00, 0x74, 0x00, 0x65, 0x00, 0x6d, 0x00, 0x57, 0x00, 0x61, 0x00,
    0x6b, 0x00, 0x65, 0x00, 0x45, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x62, 0x00, 0x6c, 0x00, 0x65, 0x00,
    0x64, 0x00, 0x00, 0x00, /* bPropertyName: SystemWakeEnabled */
    0x04, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4 */
    0x01, 0x00, 0x00, 0x00, /* bPropertyData: 0x00000001 */
#endif /* REMOTEWAKE_EN */

    /* Property section to set DeviceInterfaceGUID */
    0x84, 0x00, 0x00, 0x00, /* length */
    0x01, 0x00, 0x00, 0x00, /* dwPropertyDataType: REG_SZ */
    0x28, 0x00,             /* wPropertyNameLength: (19 + 1) * 2 = 40 */

    0x44, 0x00, 0x65, 0x00, 0x76, 0x00, 0x69, 0x00, 0x63, 0x00, 0x65, 0x00, 0x49, 0x00, 0x6E, 0x00,
    0x74, 0x00, 0x65, 0x00, 0x72, 0x00, 0x66, 0x00, 0x61, 0x00, 0x63, 0x00, 0x65, 0x00, 0x47, 0x00,
    0x55, 0x00, 0x49, 0x00, 0x44, 0x00, 0x00, 0x00, /* bPropertyName: DeviceInterfaceGUID */
    0x4E, 0x00, 0x00, 0x00, /* dwPropertyDataLength: 4E */

    '{', 0x00, '0', 0x00, '1', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, '5', 0x00, '6', 0x00,
    '7', 0x00, '-', 0x00, '2', 0x00, 'A', 0x00, '4', 0x00, 'F', 0x00, '-', 0x00, '4', 0x00,
    '9', 0x00, 'E', 0x00, 'E', 0x00, '-', 0x00, '8', 0x00, 'D', 0x00, 'D', 0x00, '3', 0x00,
    '-', 0x00, 'F', 0x00, 'A', 0x00, 'D', 0x00, 'E', 0x00, 'A', 0x00, '3', 0x00, '7', 0x00,
    '7', 0x00, '2', 0x00, '3', 0x00, '4', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00
        /* bPropertyData: {01234567-2A4F-49EE-8DD3-FADEA377234A} */
};

void CyApp_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_usb_speed_t usbSpeed)
{
    if ((pAppCtxt != NULL) && (pAppCtxt->pUsbdCtxt != NULL)) {
        DBG_APP_INFO("Setting descriptors for USB speed %x\r\n", usbSpeed);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxLangString);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxMfgString);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxProdString);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR, 0, (uint8_t *)CyFxDevQualDscr);
        Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_BOS_DSCR, 0, (uint8_t *)CyFxBOSDscr_Gen1);

        switch (usbSpeed)
        {
            case CY_USBD_USB_DEV_SS_GEN2X2:
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxBOSDscr_Gen2x2);
                break;

            case CY_USBD_USB_DEV_SS_GEN1X2:
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxBOSDscr_Gen1x2);
                break;

            case CY_USBD_USB_DEV_SS_GEN2:
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxBOSDscr_Gen2);
                break;

            default:
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxBOSDscr_Gen1);
                break;
        }
    }
}

bool CyApp_CheckConfigDescriptorLength(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    return (
            (CyFxUSBFSConfigDscr[13u] < (2u * pAppCtxt->maxLbkPairs)) &&
            (CyFxUSBHSConfigDscr[13u] < (2u * pAppCtxt->maxLbkPairs)) &&
            (CyFxUSBSSConfigDscr[13u] < (2u * pAppCtxt->maxLbkPairs))
           );
}

void CyApp_AddEpPairToCfgDscr(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_app_ep_conf_t *pEpConf)
{
    uint16_t offset;
    uint16_t bytesPerInterval;

    if (pEpConf->epType == CY_USB_ENDP_TYPE_BULK) {
        pEpConf->mPktSizeFS = (pEpConf->mPktSizeFS > 64U) ? 64U : pEpConf->mPktSizeFS;
        pEpConf->mPktSizeHS = 512U;
        pEpConf->mPktSizeSS = 1024U;
        pEpConf->maxBurst   = (pEpConf->maxBurst == 0) ? 1 : pEpConf->maxBurst;

        /* Update FS Configuration Descriptor first. */
        offset = (uint16_t)CyFxUSBFSConfigDscr[2u];         /* Identify current end of descriptor. */
        CyFxUSBFSConfigDscr[2u] += 2u * 7u;                 /* Update total descriptor length by two EP descriptors. */
        CyFxUSBFSConfigDscr[13u] += 2u;                     /* Update descriptor count by two endpoint descriptors. */

        /* Add EP descriptor for the IN endpoint. */
        CyFxUSBFSConfigDscr[offset++] = 0x07u;
        CyFxUSBFSConfigDscr[offset++] = 0x05u;
        CyFxUSBFSConfigDscr[offset++] = 0x80u | pEpConf->inEp;
        CyFxUSBFSConfigDscr[offset++] = 0x02u;              /* BULK endpoint. */
        CyFxUSBFSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeFS;
        CyFxUSBFSConfigDscr[offset++] = (pEpConf->mPktSizeFS >> 8U);
        CyFxUSBFSConfigDscr[offset++] = 0x00u;              /* No attributes for bulk EP. */

        /* Add EP descriptor for the OUT endpoint. */
        CyFxUSBFSConfigDscr[offset++] = 0x07u;
        CyFxUSBFSConfigDscr[offset++] = 0x05u;
        CyFxUSBFSConfigDscr[offset++] = pEpConf->outEp;
        CyFxUSBFSConfigDscr[offset++] = 0x02u;              /* BULK endpoint. */
        CyFxUSBFSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeFS;
        CyFxUSBFSConfigDscr[offset++] = (pEpConf->mPktSizeFS >> 8U);
        CyFxUSBFSConfigDscr[offset++] = 0x00u;              /* No attributes for bulk EP. */

        /* Update HS Configuration Descriptor. */
        offset = (uint16_t)CyFxUSBHSConfigDscr[2u];         /* Identify current end of descriptor. */
        CyFxUSBHSConfigDscr[2u] += 2u * 7u;                 /* Update total descriptor length by two EP descriptors. */
        CyFxUSBHSConfigDscr[13u] += 2u;                     /* Update descriptor count by two endpoint descriptors. */

        /* Add EP descriptor for the IN endpoint. */
        CyFxUSBHSConfigDscr[offset++] = 0x07u;
        CyFxUSBHSConfigDscr[offset++] = 0x05u;
        CyFxUSBHSConfigDscr[offset++] = 0x80u | pEpConf->inEp;
        CyFxUSBHSConfigDscr[offset++] = 0x02u;              /* BULK endpoint. */
        CyFxUSBHSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeHS;
        CyFxUSBHSConfigDscr[offset++] = (pEpConf->mPktSizeHS >> 8U);
        CyFxUSBHSConfigDscr[offset++] = 0x00u;              /* No attributes for bulk EP. */

        /* Add EP descriptor for the OUT endpoint. */
        CyFxUSBHSConfigDscr[offset++] = 0x07u;
        CyFxUSBHSConfigDscr[offset++] = 0x05u;
        CyFxUSBHSConfigDscr[offset++] = pEpConf->outEp;
        CyFxUSBHSConfigDscr[offset++] = 0x02u;              /* BULK endpoint. */
        CyFxUSBHSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeHS;
        CyFxUSBHSConfigDscr[offset++] = (pEpConf->mPktSizeHS >> 8U);
        CyFxUSBHSConfigDscr[offset++] = 0x00u;              /* No attributes for bulk EP. */

        /* Update SS Configuration Descriptor. */
        offset = ((uint16_t *)CyFxUSBSSConfigDscr)[1];      /* Identify current end of descriptor. */
        if ((CyFxUSBSSConfigDscr[2u] + (2u * (7u + 6u))) > 255u) {
            CyFxUSBSSConfigDscr[2u] += (2u * (7u + 6u));    /* Update total descriptor length by two EP descriptors. */
            CyFxUSBSSConfigDscr[3u]++;
        } else {
            CyFxUSBSSConfigDscr[2u] += (2u * (7u + 6u));    /* Update total descriptor length by two EP descriptors. */
        }
        CyFxUSBSSConfigDscr[13u] += 2u;                     /* Update descriptor count by two endpoint descriptors. */

        /* Add EP descriptor for the IN endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x07u;
        CyFxUSBSSConfigDscr[offset++] = 0x05u;
        CyFxUSBSSConfigDscr[offset++] = 0x80 | pEpConf->inEp;
        CyFxUSBSSConfigDscr[offset++] = 0x02u;              /* BULK endpoint. */
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeSS;
        CyFxUSBSSConfigDscr[offset++] = (pEpConf->mPktSizeSS >> 8U);
        CyFxUSBSSConfigDscr[offset++] = 0x00u;              /* No attributes for bulk EP. */
        /* Add EP Companion descriptor for the IN endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x06u;
        CyFxUSBSSConfigDscr[offset++] = 0x30u;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->maxBurst - 1;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->ssAttrib;
        CyFxUSBSSConfigDscr[offset++] = 0x00u;
        CyFxUSBSSConfigDscr[offset++] = 0x00u;

        /* Add EP descriptor for the OUT endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x07u;
        CyFxUSBSSConfigDscr[offset++] = 0x05u;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->outEp;
        CyFxUSBSSConfigDscr[offset++] = 0x02u;              /* BULK endpoint. */
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeSS;
        CyFxUSBSSConfigDscr[offset++] = (pEpConf->mPktSizeSS >> 8U);
        CyFxUSBSSConfigDscr[offset++] = 0x00u;              /* No attributes for bulk EP. */
        /* Add EP Companion descriptor for the OUT endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x06u;
        CyFxUSBSSConfigDscr[offset++] = 0x30u;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->maxBurst - 1;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->ssAttrib;
        CyFxUSBSSConfigDscr[offset++] = 0x00u;
        CyFxUSBSSConfigDscr[offset++] = 0x00u;
    } else {
        /* ISOCHRONOUS/INTERRUPT EP SUPPORT. */
        if (pEpConf->epType == CY_USB_ENDP_TYPE_INTR) {
            pEpConf->mPktSizeFS = (pEpConf->mPktSizeFS > 64U) ? 64U : pEpConf->mPktSizeFS;
            pEpConf->mPktSizeHS = (pEpConf->mPktSizeHS & 0x1FFFU);
            pEpConf->mPktSizeSS = (pEpConf->mPktSizeSS > 1024U) ? 1024U : pEpConf->mPktSizeSS;
            pEpConf->maxBurst   = (pEpConf->maxBurst == 0) ? 1 : pEpConf->maxBurst;
            pEpConf->ssAttrib   = 0;
        } else {
            pEpConf->mPktSizeFS = (pEpConf->mPktSizeFS > 1023U) ? 1023U : pEpConf->mPktSizeFS;
            pEpConf->mPktSizeHS = (pEpConf->mPktSizeHS & 0x1FFFU);
            pEpConf->mPktSizeSS = (pEpConf->mPktSizeSS > 1024U) ? 1024U : pEpConf->mPktSizeSS;
            pEpConf->maxBurst   = (pEpConf->maxBurst == 0) ? 1 : pEpConf->maxBurst;
            pEpConf->ssAttrib   = (pEpConf->ssAttrib > 2) ? 2 : pEpConf->ssAttrib;
        }

        bytesPerInterval = (pEpConf->mPktSizeSS * pEpConf->maxBurst * (pEpConf->ssAttrib + 1));

        /* Update FS Configuration Descriptor first. */
        offset = (uint16_t)CyFxUSBFSConfigDscr[2u];         /* Identify current end of descriptor. */
        CyFxUSBFSConfigDscr[2u] += 2u * 7u;                 /* Update total descriptor length by two EP descriptors. */
        CyFxUSBFSConfigDscr[13u] += 2u;                     /* Update descriptor count by two endpoint descriptors. */

        /* Add EP descriptor for the IN endpoint. */
        CyFxUSBFSConfigDscr[offset++] = 0x07u;
        CyFxUSBFSConfigDscr[offset++] = 0x05u;
        CyFxUSBFSConfigDscr[offset++] = 0x80u | pEpConf->inEp;
        CyFxUSBFSConfigDscr[offset++] = (uint8_t)pEpConf->epType;
        CyFxUSBFSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeFS;
        CyFxUSBFSConfigDscr[offset++] = (pEpConf->mPktSizeFS >> 8U);
        CyFxUSBFSConfigDscr[offset++] = pEpConf->pollRate;

        /* Add EP descriptor for the OUT endpoint. */
        CyFxUSBFSConfigDscr[offset++] = 0x07u;
        CyFxUSBFSConfigDscr[offset++] = 0x05u;
        CyFxUSBFSConfigDscr[offset++] = pEpConf->outEp;
        CyFxUSBFSConfigDscr[offset++] = (uint8_t)pEpConf->epType;
        CyFxUSBFSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeFS;
        CyFxUSBFSConfigDscr[offset++] = (pEpConf->mPktSizeFS >> 8U);
        CyFxUSBFSConfigDscr[offset++] = pEpConf->pollRate;

        /* Update HS Configuration Descriptor. */
        offset = (uint16_t)CyFxUSBHSConfigDscr[2u];         /* Identify current end of descriptor. */
        CyFxUSBHSConfigDscr[2u] += 2u * 7u;                 /* Update total descriptor length by two EP descriptors. */
        CyFxUSBHSConfigDscr[13u] += 2u;                     /* Update descriptor count by two endpoint descriptors. */

        /* Add EP descriptor for the IN endpoint. */
        CyFxUSBHSConfigDscr[offset++] = 0x07u;
        CyFxUSBHSConfigDscr[offset++] = 0x05u;
        CyFxUSBHSConfigDscr[offset++] = 0x80u | pEpConf->inEp;
        CyFxUSBHSConfigDscr[offset++] = (uint8_t)pEpConf->epType;
        CyFxUSBHSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeHS;
        CyFxUSBHSConfigDscr[offset++] = (pEpConf->mPktSizeHS >> 8U);
        CyFxUSBHSConfigDscr[offset++] = pEpConf->pollRate;

        /* Add EP descriptor for the OUT endpoint. */
        CyFxUSBHSConfigDscr[offset++] = 0x07u;
        CyFxUSBHSConfigDscr[offset++] = 0x05u;
        CyFxUSBHSConfigDscr[offset++] = pEpConf->outEp;
        CyFxUSBHSConfigDscr[offset++] = (uint8_t)pEpConf->epType;
        CyFxUSBHSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeHS;
        CyFxUSBHSConfigDscr[offset++] = (pEpConf->mPktSizeHS >> 8U);
        CyFxUSBHSConfigDscr[offset++] = pEpConf->pollRate;

        /* Update SS Configuration Descriptor. */
        offset = ((uint16_t *)CyFxUSBSSConfigDscr)[1];      /* Identify current end of descriptor. */
        if ((CyFxUSBSSConfigDscr[2u] + (2u * (7u + 6u))) > 255u) {
            CyFxUSBSSConfigDscr[2u] += (2u * (7u + 6u));    /* Update total descriptor length by two EP descriptors. */
            CyFxUSBSSConfigDscr[3u]++;
        } else {
            CyFxUSBSSConfigDscr[2u] += (2u * (7u + 6u));    /* Update total descriptor length by two EP descriptors. */
        }
        CyFxUSBSSConfigDscr[13u] += 2u;                     /* Update descriptor count by two endpoint descriptors. */

        /* Add EP descriptor for the IN endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x07u;
        CyFxUSBSSConfigDscr[offset++] = 0x05u;
        CyFxUSBSSConfigDscr[offset++] = 0x80 | pEpConf->inEp;
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)pEpConf->epType;
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeSS;
        CyFxUSBSSConfigDscr[offset++] = (pEpConf->mPktSizeSS >> 8U);
        CyFxUSBSSConfigDscr[offset++] = pEpConf->pollRate;
        /* Add EP companion descriptor for the IN endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x06u;              /* Length of EP companion descriptor. */
        CyFxUSBSSConfigDscr[offset++] = 0x30u;              /* Type is EP companion descriptor. */
        CyFxUSBSSConfigDscr[offset++] = pEpConf->maxBurst - 1;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->ssAttrib;
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)bytesPerInterval;
        CyFxUSBSSConfigDscr[offset++] = (bytesPerInterval >> 8U);

        /* Add EP descriptor for the OUT endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x07u;
        CyFxUSBSSConfigDscr[offset++] = 0x05u;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->outEp;
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)pEpConf->epType;
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)pEpConf->mPktSizeSS;
        CyFxUSBSSConfigDscr[offset++] = (pEpConf->mPktSizeSS >> 8U);
        CyFxUSBSSConfigDscr[offset++] = pEpConf->pollRate;
        /* Add EP companion descriptor for the OUT endpoint. */
        CyFxUSBSSConfigDscr[offset++] = 0x06u;              /* Length of EP companion descriptor. */
        CyFxUSBSSConfigDscr[offset++] = 0x30u;              /* Type is EP companion descriptor. */
        CyFxUSBSSConfigDscr[offset++] = pEpConf->maxBurst - 1;
        CyFxUSBSSConfigDscr[offset++] = pEpConf->ssAttrib;
        CyFxUSBSSConfigDscr[offset++] = (uint8_t)bytesPerInterval;
        CyFxUSBSSConfigDscr[offset++] = (bytesPerInterval >> 8U);
    }
}

void CyApp_RevertConfigDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    (void)pAppCtxt;

    CyFxUSBFSConfigDscr[2u]    = 0x12u;
    CyFxUSBFSConfigDscr[3u]    = 0x12u;
    CyFxUSBFSConfigDscr[13u]   = 0x00u;

    CyFxUSBHSConfigDscr[2u]    = 0x12u;
    CyFxUSBHSConfigDscr[3u]    = 0x12u;
    CyFxUSBHSConfigDscr[13u]   = 0x00u;

    CyFxUSBSSConfigDscr[2u]    = 0x12u;
    CyFxUSBSSConfigDscr[3u]    = 0x00u;
    CyFxUSBSSConfigDscr[13u]   = 0x00u;
}

void CyApp_SetProductId(cy_stc_usb_app_ctxt_t *pAppCtxt, uint16_t pidVal)
{
    (void)pAppCtxt;

    CyFxUSB20DeviceDscr[10] = (uint8_t)(pidVal & 0xFFU);
    CyFxUSB20DeviceDscr[11] = (uint8_t)(pidVal >> 8U);
    CyFxUSB30DeviceDscr[10] = (uint8_t)(pidVal & 0xFFU);
    CyFxUSB30DeviceDscr[11] = (uint8_t)(pidVal >> 8U);
}

/*[]*/

