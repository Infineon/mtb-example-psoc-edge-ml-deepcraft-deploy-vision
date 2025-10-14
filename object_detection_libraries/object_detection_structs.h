/***************************************************************************//**
* \file object_detection_structs.h
*
* \brief
* This is the header file of object detection utility C structs.
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
#ifndef _DETECTION_H_
#define _DETECTION_H_

#include <stdint.h>

#include "cy_result.h"
#include "definitions.h"
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#ifndef max
    #define max(a, b)   ((a) > (b) ? (a) : (b))
    #define min(a, b)   ((a) < (b) ? (a) : (b))
#endif  // max

#define QUANTIZE( r, zero_point, scale )    ((int)((r) / (scale) + 0.5f) + (zero_point))
#define DEQUANTIZE( q, zero_point, scale )  (((int)(q) - (zero_point)) * (scale))

/******************************************************************************
 * Typedefs
 *****************************************************************************/
/**
 *
 */
/***************************************************************************//**
* \struct ImageProperty_t
*
* \brief
* Input image structure.
*
* \details
* Detailed description of the struct's purpose and usage (optional)
*
********************************************************************************/
/**
 * detection option structure
 */
typedef struct detection_opt_st
{
    uint8_t *image_buf;
    float   image_width;                // original width of image
    float   image_height;               // original height of image
    float   threshold_score;            // threshold for confidence score
    float   threshold_IOU;              // threshold for IoU (intersection over union) in NMS
    int32_t     max_detections;         // maximum number of predictions
} detection_opt_t;

extern detection_opt_t Opt;


typedef struct model_detail {
    int8_t *    data;
    size_t      size;
    int *       dim;
    int         dim_len;
    int         zero;
    float       scale;
} model_detail_t;


// final output variables
typedef struct {
    int32_t     count;
    int16_t     bbox_int16[Max_detections * 4];
    float       conf[Max_detections];
    uint8_t     class_id[Max_detections];
} prediction_OD_t;

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif  // _DETECTION_H_
