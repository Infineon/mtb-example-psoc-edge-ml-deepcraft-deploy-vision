/***************************************************************************//**
* \file definitions.h
*
* \brief
* This is the header file of object detection utility definitions and constants.
*
********************************************************************************
* (c) 2023-2025, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
* 
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#ifndef _DEFINITIONS_H_
#define _DEFINITIONS_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>


/*******************************************************************************
* Macros
*******************************************************************************/
#define CAMERA_WIDTH    320     // default USB camera input dimensions
#define CAMERA_HEIGHT   240

#define IMAGE_WIDTH     320     // model input dimensions, hard coded for now to help on LCD graphics maintain an image buffer of model size
#define IMAGE_HEIGHT    320

#define FRAMES_TO_SKIP 2        // if USB webcam stream is sharding, skip some frames (inference every FRAMES_TO_SKIP frames)
#define FRAMES_TO_SKIP_LOGITECH 4

// Object Detection Configuration
#define Max_detections  2       // number of max predictions
#define Thresh_conf     0.7     // confidence threshold
#define Thresh_IoU      0.3     // IoU threshold

#define NPU_PRIORITY    3
#define N_OD_INPUT      1       // model has one input layers
#define N_OD_OUTPUT     1       // model has one output

#ifndef max
    #define max(a, b)   ((a) > (b) ? (a) : (b))
    #define min(a, b)   ((a) < (b) ? (a) : (b))
#endif

#endif  // _DEFINITIONS_H_
