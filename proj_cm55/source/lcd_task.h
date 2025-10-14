/***************************************************************************//**
* \file lcd_task.h
*
* \brief
* This is the header file of LCD display functions.
*
********************************************************************************
* (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
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
 * Header Guards
 *******************************************************************************/
#ifndef _LCD_TASK_H_
#define _LCD_TASK_H_

/*******************************************************************************
 * Included Headers
 *******************************************************************************/
#ifdef __cplusplus
extern "C" {
#endif

#include "cyabs_rtos.h"
#include "vg_lite.h"
#include "vg_lite_platform.h"
#include "object_detection_lib.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define NUM_IMAGE_BUFFERS  2  // Number of image buffers

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
extern cy_semaphore_t model_semaphore;  // Model semaphore for synchronization
extern int8_t bgr888_int8[];           // BGR888 integer buffer

/*******************************************************************************
 * Local Variables
 *******************************************************************************/
extern vg_lite_buffer_t *renderTarget;         // Render target buffer
extern vg_lite_buffer_t usb_yuv_frames[];      // USB YUV frame buffers

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
/*******************************************************************************
 * Function Name: draw
 *******************************************************************************
 *
 * Summary:
 *  Draws the image using GPU and refreshes the frame buffer. Selects a ready
 *  camera input image buffer and converts the input image in YUV422 to RGB888.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  Pointer to the image buffer in RGB888.
 *
 *******************************************************************************/
int8_t *draw(void);

/*******************************************************************************
 * Function Name: update_box_data
 *******************************************************************************
 *
 * Summary:
 *  Draws bounding boxes and text annotations on the specified frame buffer.
 *
 * Parameters:
 *  renderTarget - Pointer to the background frame buffer
 *  prediction   - Pointer to the prediction structure
 *
 * Return:
 *  Number of meaningful confidence scores
 *
 *******************************************************************************/
void update_box_data(vg_lite_buffer_t *renderTarget, prediction_OD_t *prediction);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif //_LCD_TASK_H_
