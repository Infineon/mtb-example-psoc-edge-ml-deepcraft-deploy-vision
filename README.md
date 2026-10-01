# PSOC&trade; Edge MCU: Machine learning – DEEPCRAFT&trade; deploy vision

This code example demonstrates a real-time hand gesture detection that uses a USB camera to capture live video and a DEEPCRAFT&trade; Studio object detection model to detect hand gestures (rock, paper, or scissors) in the video feed using ModusToolbox&trade;. The detected gestures are highlighted by drawing a bounding box around the gesture and displaying the corresponding text (rock, paper, or scissors) in a text box on the display and on a terminal.

This code example has a three-project structure: CM33 secure, CM33 non-secure, and CM55 projects. All three projects are programmed to the external QSPI flash and executed in Execute in Place (XIP) mode. Extended boot launches the CM33 secure project from a fixed location in the external flash, which then configures the protection settings and launches the CM33 non-secure application. Additionally, CM33 non-secure application enables CM55 CPU and launches the CM55 application.
> **Note:** On the KIT_PSE84_HMI, all three projects are programmed to the external OSPI flash instead of QSPI.

[View this README on GitHub.](https://github.com/Infineon/mtb-example-psoc-edge-ml-deepcraft-deploy-vision)

[Provide feedback on this code example.](https://yourvoice.infineon.com/jfe/form/SV_1NTns53sK2yiljn?Q_EED=eyJVbmlxdWUgRG9jIElkIjoiQ0UyNDIxNDUiLCJTcGVjIE51bWJlciI6IjAwMi00MjE0NSIsIkRvYyBUaXRsZSI6IlBTT0MmdHJhZGU7IEVkZ2UgTUNVOiBNYWNoaW5lIGxlYXJuaW5nIOKAkyBERUVQQ1JBRlQmdHJhZGU7IGRlcGxveSB2aXNpb24iLCJyaWQiOiJzYW5kZWVwLmFrQGluZmluZW9uLmNvbSIsIkRvYyB2ZXJzaW9uIjoiMi40LjEiLCJEb2MgTGFuZ3VhZ2UiOiJFbmdsaXNoIiwiRG9jIERpdmlzaW9uIjoiTUNEIiwiRG9jIEJVIjoiSUNXIiwiRG9jIEZhbWlseSI6IlBTT0MifQ==)

See the [Design and implementation](docs/design_and_implementation.md) for the functional description of this code example.


## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.7 or later (tested with v3.8)
- Board support package (BSP) minimum required version: 1.0.0
- Programming language: C
- Associated parts: All [PSOC&trade; Edge MCU](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm) parts


## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v14.2.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- LLVM Embedded Toolchain for Arm&reg; v19.1.5 (`LLVM_ARM`)

> **Notes:**
> - IAR is not supported by the TensorFlow Lite for Microcontrollers (TFLM) library
> - This code example fails to build in RELEASE mode with the `GCC_ARM` toolchain v14.2.1 as it does not recognize some of the Helium instructions of the CMSIS-DSP library. This issue is not present in the Arm&reg; Compiler for Embedded (armclang)
> - This code example currently supports the VFP_SELECT option "hardfp" only. Setting "softfp" may cause build failure.


## Supported kits (make variable 'TARGET')

- [PSOC&trade; Edge E84 Evaluation Kit](https://www.infineon.com/KIT_PSE84_EVAL) (`KIT_PSE84_EVAL_EPC2`) – Default value of `TARGET`
- [PSOC&trade; Edge E84 AI Kit](https://www.infineon.com/KIT_PSE84_AI) (`KIT_PSE84_AI`)
- [PSOC&trade; Edge E84 HMI Kit](https://www.infineon.com/KIT_PSE84_HMI) (`KIT_PSE84_HMI`)


## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

Ensure the following jumper and pin configuration on board.
- BOOT SW must be in the HIGH/ON position
- J20 and J21 must be in the tristate/not connected (NC) position for the PSOC&trade; Edge E84 Evaluation Kit

> **Note:** This hardware setup is not required for the KIT_PSE84_AI kit.

While using DVP Camera with PSOC&trade; Edge E84 AI Kit, refer to the [PSOC&trade; Edge E84 AI Kit guide](https://www.infineon.com/assets/row/public/documents/30/44/infineon-kit-pse84-ai-user-guide-usermanual-en.pdf) for instructions on connecting the camera module.

### Supported camera and display

- Connect any of the following cameras to the USB host port on the kit
  - [HBVCAM OV7675 0.3MP MINI Camera](https://www.hbvcamera.com/0-3mp-pixel-usb-cameras/hbvcam-ov7675-0.3mp-mini-laptop-camera-module.html) <br>
  - [Logitech C920 HD Pro Webcam](https://www.logitech.com/en-ch/shop/p/c920-pro-hd-webcam) <br>
  - [Logitech C920e Business Webcam](https://www.logitech.com/en-ch/products/webcams/c920e-business-webcam) <br>
  - [HBVCAM OS02F10 2MP Camera](https://www.hbvcamera.com/2-mega-pixel-usb-cameras/2mp-1080p-auto-focus-hd-usb-camera-module-for-atm-machine.html) <br>

- The PSOC&trade; Edge AI Kit also supports -
  - [OV7675 0.3MP DVP Camera module](https://blog.arducam.com/products/camera-breakout-board/0-3mp-ov7675) <br>

  > **Note:** To enable the DVP Camera, update the value of `CAMERA_TYPE` variable in the Makefile of CM55 project from `CAM_USB` to `CAM_DVP`.

> **Note:** For any USB camera other than the ones listed above, ensure that the vendor ID  and product ID of the camera being used are correctly configured in the *usb_camera_task.c* and *usb_camera_task.h* file.

- **Waveshare 4.3 inch Raspberry Pi DSI 800 x 480 display:** <br>
  Connect the FPC 15-pin cable between the display connector and the PSOC&trade; Edge E84's RPI MIPI DSI connector as outlined in **Table 1** and **Figure 1**

   **Table 1. Cable connection between display connect and kit**

   Kit's name                                      | DSI connector
   ----------------------------------------------- | --------------
   PSOC&trade; Edge E84 Evaluation Kit             | J39
   PSOC&trade; Edge E84 AI Kit                     | J10

   <br>

  **Figure 1. Display connection with PSOC&trade; Edge E84 Evaluation Kit**

  ![](images/display-kit-connection.png)

- **ST7701S 4-inch MIPI DSI 512x480 pixel display (RK040HF001):** This display is by default
  shipped with PSOC&trade; Edge E84 HMI Kit.

   **Figure 2. 4-inch MIPI DSI 512x480 pixel display**

   ![](images/hmi_kit_image.png)


## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.

Install a terminal emulator if you do not have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).

Install the [ModusToolbox&trade; Machine Learning Pack](https://softwaretools.infineon.com/tools/com.ifx.tb.tool.modustoolboxpackmachinelearning) or use the [Infineon Developer Center (IDC)](https://www.infineon.com/cms/en/design-support/tools/utilities/infineon-developer-center-idc-launcher/) launcher and search for "ModusToolbox Machine Learning Pack" and install it.

This example requires no additional software or tools.

> **Note:** This code example currently does not work with the custom BSP name for the `KIT_PSE84_EVAL_EPC2`, `KIT_PSE84_AI` and `KIT_PSE84_HMI` kits. If you want to change the BSP name to a non-default value, ensure to update the custom BSP name in *Makefile* under the relevant section. The build fails if you do not update the custom BSP name.


## Operation

See [Using the code example](docs/using_the_code_example.md) for instructions on creating a project, opening it in various supported IDEs, and performing tasks, such as building, programming, and debugging the application within the respective IDEs.

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud

3. Connect the USB camera to the kit's USB host port and display as mentioned in the [Supported camera and display](#supported-camera-and-display) section

4. After programming, the application starts automatically. Confirm that "PSOC Edge MCU: Machine learning DEEPCRAFT deploy vision" is displayed on the UART terminal and the kit will start capturing video from the USB camera

   **Figure 2. Terminal output on program startup for deploy vision**

   ![](images/terminal-deploy-vision.png)

5. Perform hand gestures (rock, paper, or scissors) in front of the camera

6. The display will show the live video feed with bounding boxes and labels for detected gestures

   **Figure 3. Display output for recognized hand gestures**

   **Rock gesture detection** | **Paper gesture detection** | **Scissors gesture detection**
   ------------------------| ------------------------| ------------------------
   ![](images/rock-detection.gif) | ![](images/paper-detection.gif) | ![](images/scissors-detection.gif)

  <br>

7. Labels for the detected hand gesture is displayed on the UART terminala

   **Figure 4. Terminal output on gesture detection for deploy vision**

   ![](images/terminal-gesture-detection.png)

  <br>

The code example can be updated to use other starter models from DEEPCRAFT&trade; Studio. For more information, see [Deploy vision model on PSOC&trade; 6 and PSOC&trade; Edge boards](https://developer.imagimob.com/deepcraft-studio/deployment/deploy-models-supported-boards/deploy-vision-model-PSOC-Edge). For details on generating, optimizing, and validating the model code using DEEPCRAFT&trade; Studio, see [Code generation for Infineon boards](https://developer.imagimob.com/deepcraft-studio/code-generation/code-gen-infineon-boards).

> **Note:** The vision model provided in this example is not production ready and is provided here for reference purpose only. You can develop your own models in DEEPCRAFT&trade; Studio and use this example to deploy them on PSOC&trade; Edge MCU by replacing the corresponding .c/.h files.

## Related resources

Resources  | Links
-----------|----------------------------------
Application notes  | [AN235935](https://www.infineon.com/AN235935) – Getting started with PSOC&trade; Edge E8 MCU on ModusToolbox&trade; software
Code examples  | [Using ModusToolbox&trade;](https://github.com/Infineon/Code-Examples-for-ModusToolbox-Software) on GitHub
Device documentation | [PSOC&trade; Edge MCU datasheets](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm#documents) <br> [PSOC&trade; Edge MCU reference manuals](https://www.infineon.com/products/microcontroller/32-bit-psoc-arm-cortex/32-bit-psoc-edge-arm#documents)
Development kits | Select your kits from the [Evaluation board finder](https://www.infineon.com/cms/en/design-support/finder-selection-tools/product-finder/evaluation-board)
Libraries  | [mtb-dsl-pse8xxgp](https://github.com/Infineon/mtb-dsl-pse8xxgp) – Device support library for PSE8XXGP <br> [retarget-io](https://github.com/Infineon/retarget-io) – Utility library to retarget STDIO messages to a UART port
Tools  | [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) – ModusToolbox&trade; software is a collection of easy-to-use libraries and tools enabling rapid development with Infineon MCUs for applications ranging from wireless and cloud-connected systems, edge AI/ML, embedded sense and control, to wired USB connectivity using PSOC&trade; Industrial/IoT MCUs, AIROC&trade; Wi-Fi and Bluetooth&reg; connectivity devices, XMC&trade; Industrial MCUs, and EZ-USB&trade;/EZ-PD&trade; wired connectivity controllers. ModusToolbox&trade; incorporates a comprehensive set of BSPs, HAL, libraries, configuration tools, and provides support for industry-standard IDEs to fast-track your embedded application development

<br>


## Other resources

Infineon provides a wealth of data at [www.infineon.com](https://www.infineon.com) to help you select the right device, and quickly and effectively integrate it into your design.


## Document history

Document title: *CE242145* – *PSOC&trade; Edge MCU: DEEPCRAFT&trade; deploy vision*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | New code example
 2.0.0   | Updated to work with latest DEEPCRAFT&trade;
 2.0.1   | README update to include instructions for enabling DVP Camera
 2.1.0   | Updated design files to fix ModusToolbox&trade; v3.7 build warnings
 2.2.0   | Enabled 24 MHz EXT_CLK to reduce USB camera transaction errors
 2.3.0   | Added support for KIT_PSE84_HMI
 2.4.0   | Updated to auto-calculate the class count at runtime from model metadata <br> Added support for automatic preprocessing (letterbox/crop) and bounding box mapping based on model type <br> Added support for custom model input sizes <br> Updated linker scripts for GCC_ARM toolchain
 2.4.1   | ECO configurations update for KIT_PSE84_HMI
<br>


All referenced product or service names and trademarks are the property of their respective owners.

The Bluetooth&reg; word mark and logos are registered trademarks owned by Bluetooth SIG, Inc., and any use of such marks by Infineon is under license.

PSOC&trade;, formerly known as PSoC&trade;, is a trademark of Infineon Technologies. Any references to PSoC&trade; in this document or others shall be deemed to refer to PSOC&trade;.

---------------------------------------------------------

(c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
This software, associated documentation and materials ("Software") is owned by Infineon Technologies AG or one of its affiliates ("Infineon") and is protected by and subject to worldwide patent protection, worldwide copyright laws, and international treaty provisions. Therefore, you may use this Software only as provided in the license agreement accompanying the software package from which you obtained this Software. If no license agreement applies, then any use, reproduction, modification, translation, or compilation of this Software is prohibited without the express written permission of Infineon.
<br>
Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR MERCHANTABILITY. Infineon reserves the right to make changes to the Software without notice. You are responsible for properly designing, programming, and testing the functionality and safety of your intended application of the Software, as well as complying with any legal requirements related to its use. Infineon does not guarantee that the Software will be free from intrusion, data theft or loss, or other breaches (“Security Breaches”), and Infineon shall have no liability arising out of any Security Breaches. Unless otherwise explicitly approved by Infineon, the Software may not be used in any application where a failure of the Product or any consequences of the use thereof can reasonably be expected to result in personal injury.
