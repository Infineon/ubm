# UBM Middleware Library 1.0

## What's Included?

For a complete description of the UBM Middleware, refer to the [README.md](./README.md) and [API Reference Guide](https://infineon.github.io/ubm/html/index.html).

The UBM middleware 1.0 release implements the following features:

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
- DFC Status and Control Descriptors
- Port Route Descriptors
- Change Count command
- DFC Status and Control Descriptor Index command
- DFC Status and Control Descriptor command
- Features write command
## Known issues/limitations

 The release does not support the features to be implemented in the future releases:

- FRU write 2wire transactions

 The UBM Middleware was tested with the following drive types only:
- SFF-TA-1001 PCIe (U.3)
- SAS/SATA
- Quad PCIe (U.2)

## Defect Fixes

## Supported Software and Tools

This version of the UBM Middleware was validated for the compatibility with the following Software and Tools:

| Software and Tools                                      | Version |
| :---                                                    | :----:  |
| ModusToolbox Software Environment                       | 3.1.0   |
| CAT1 Peripheral Driver Library                          | 3.3.0   |
| CAT1 Hardware Abstraction Layer                         | 2.3.0   |
| MCUBoot                                                 | 1.8.3   |
| emeeprom                                                | 2.20.0  |
| Core Library                                            | 1.3.1   |
| GCC Compiler                                            | 11.3.1  |
| IAR Compiler                                            | 8.42    |
| ARM Compiler 6                                          | 6.13    |

## More information

For more information, refer to the following documents:

- [UBM Middleware README.md](./README.md)
- [UBM Middleware API Reference Guide](https://infineon.github.io/ubm/html/index.html)
- [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/)
- [Infineon Technologies AG](https://www.infineon.com)

---
© 2022-2023, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
