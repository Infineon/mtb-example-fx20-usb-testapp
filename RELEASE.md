# EZ-USB&trade; FX20: USB test application v1.0.1

## What's Included?

Refer to the [README.md](./README.md).

## Feature Changes

* Updated application to make use of USBFXStack version 1.3.0
* Added linker scripts and instructions to port the application to any EZ-USB&trade; FX device

## Defect Fixes

* Fixed USBCV test failure by avoiding the use of Interrupt and Isochronous endpoints when LPM feature is enabled

## Supported Software and Tools

This version of the application is compatible with the following software and tools:

| Software and Tools                                       | Version |
| :---                                                     | :----:  |
| ModusToolbox&trade; software environment                 | 3.5.0   |
| CAT1A Peripheral Driver Library                          | 3.16.0  |
| USBFXStack Middleware Library                            | 1.3.0   |
| FreeRTOS&trade; for Infineon MCUs                        | 10.5.004|
| GNU Arm&reg; Embedded Compiler                           | 11.3.1  |
| Arm&reg; Compiler                                        | 6.22    |

## More information

For more information, refer to the following documents:

* [EZ-USB&trade; FX20: USB test application README.md](./README.md)
* [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software)
* [Infineon Technologies AG](https://www.infineon.com)

---
© 2024, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
