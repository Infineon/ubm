# UBM Middleware Library 0.5

## What's Included?

Refer to the [README.md](./README.md) and the [API Reference Guide](https://infineon.github.io/ubm/html/index.html) for a complete description of the UBM Middleware.

## Known issues/limitations
UBM middleware 0.5 release implements next features:
 - Programmable firmware update
 - FRU read 2wire transactions
 - Operational State command
 - Last Command Status command
 - Silicon Identity and Version command
 - Programming Update Mode Capabilities command
 - Host Facing Connector Info command
 - Backplane Info command
 - Starting Slot command
 - Capabilities command
 - Features read command

 The release does not support next features, which will be implemented in next releases:
 - DFC Status and Control Descriptors
 - Port Route Descriptors
 - Change Count command
 - DFC Status and Control Descriptor Index command
 - DFC Status and Control Descriptor command
 - Features write command
 - FRU write 2wire transactions

UBM middleware 0.5 release requires a 100 MHz system High-Frequency Clock value setting for its proper operation.

Please allow at least 30 milliseconds for the PROGRAM subcommand to finish flash write operations before the following PROGRAM STATUS subcommand.

## Defect Fixes

* Initial release

## Supported Software and Tools

This version of the UBM Middleware was validated for the compatibility with the following Software and Tools:

| Software and Tools                                      | Version |
| :---                                                    | :----:  |
| ModusToolbox Software Environment                       | 3.0.0   |
| CAT1 Peripheral Driver Library                          | 3.3.0   |
| CAT1 Hardware Abstraction Layer                         | 2.3.0   |
| MCUBoot                                                 | 1.8.3   |
| emeeprom                                                | 2.20.0  |
| Core Library                                            | 1.3.1   |
| GCC Compiler                                            | 10.3.1  |
| IAR Compiler                                            | 8.42    |
| ARM Compiler 6                                          | 6.13    |

## More information

For more information, refer to the following documents:

* [UBM Middleware README.md](./README.md)
* [UBM Middleware API Reference Guide](https://infineon.github.io/ubm/html/index.html)
* [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/)
* [Infineon Technologies AG](https://www.infineon.com)

---
© 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
