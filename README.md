# UBM middleware

## Overview

The the implementation of the UBM middleware for ModusToolbox&trade; is currently 
at the Preliminary level. Features may change without notice.
Contact <a href="https:/\/www.infineon.com/cms/en/about-infineon/company/contacts/support/">
<b>Infineon Product Support</b></a> for additional details.

The UBM middleware provides the implementation of the Universal Backplane Management (UBM) - a common backplane management framework
for a host to determine SAS/SATA/PCIe backplane capabilities, Drive Facing Connector (DFC) Status and Control information, and to read
the port routing of the Drive Facing Connectors to Host Facing Connectors (HFC) of the backplane.

## Features

 - Status and Control over Drive Facing Connector I/O
 - High speed lane port routing assignments to Host Facing Connectors
 - Backplane capabilities including:
     - PCIe Reference Clock expectations
     - PCIe Reset expectations
     - Power Disable support
     - Dual Port support
 - Controller programmable firmware update

The UBM requires: 
 - [emeeprom](https://github.com/Infineon/emeeprom) v2.20.0.
 - [MCUBoot](https://github.com/mcu-tools/mcuboot) v1.8.3.

See the [UBM middleware release notes](./RELEASE.md) for release-specific information.

## Quick Start

Refer to the [API Reference Guide Quick Start Guide](https://infineon.github.io/ubm/html/index.html) section for step-by-step instruction how to enable the UBM middleware.

## More information

For more information, refer to the following documents:

* [UBM middleware release notes](./RELEASE.md)
* [SFF-TA-1005](https://members.snia.org/document/dl/27167)
* [UBM middleware API reference manual](https://infineon.github.io/ubm/html/index.html)
* [ModusToolbox Software Environment, Quick Start Guide, Documentation, and Videos](https://www.infineon.com/cms/en/design-support/tools/sdk/modustoolbox-software/)
* [Infineon Technologies AG](https://www.infineon.com)
* [Code Examples for ModusToolbox Software](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software)
* [ModusToolbox Device Configurator Tool Guide](https://www.infineon.com/ModusToolboxDeviceConfig)
* [PSoC 6 MCU Datasheets](https://www.infineon.com/cms/en/search.html#!view=downloads&term=psoc6&doc_group=Data%20Sheet)
* [PSoC 6 MCU Technical Reference Manuals](https://www.infineon.com/cms/en/search.html#!view=downloads&term=psoc6&doc_group=Additional%20Technical%20Information)

---
© 2023, Cypress Semiconductor Corporation (an Infineon company) or an affiliate of Cypress Semiconductor Corporation.
