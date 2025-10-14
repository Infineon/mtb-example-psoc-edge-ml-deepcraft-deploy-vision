 /******************************************************************************
* File Name: inference_task.c
*
* Description: This file contains API calls to inference task.
*
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
* Header File
*******************************************************************************/
#include <math.h>
#include <inttypes.h>
#include "cybsp.h"
#include "cy_pdl.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
/* FreeRTOS header file */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "FreeRTOSConfig.h"
#include "stdlib.h"
#include "definitions.h"
#include "object_detection_lib.h"
#include "ifx_image_utils.h"
#include "lcd_task.h"
#include "inference_task.h"
#include "ifx_time_utils.h"
#include "mtb_ml_utils.h"
#include "mtb_ml_common.h"
#include "mtb_ml.h"

/* Include object detection models */
#include MTB_ML_INCLUDE_MODEL_FILE(YOLOV5N)
mtb_ml_model_bin_t  model_bin = {MTB_ML_MODEL_BIN_DATA(YOLOV5N)};

/*******************************************************************************
 * Global Variable
 *******************************************************************************/
float Prep_Wait_Buf;               // Performance measure: Buffer wait time
float Prep_YUV422_to_bgr565;       // Performance measure: YUV422 to BGR565 conversion time
float Prep_bgr565_to_Disp;         // Performance measure: BGR565 to display time
float Prep_RGB565_to_RGB888;       // Performance measure: RGB565 to RGB888 conversion time

__attribute__((section(".cy_socmem_data")))
__attribute__((aligned(16))) prediction_OD_t Prediction; // Final output variable for object detection

volatile float Inference_time = 0;  // Inference execution time

extern cy_semaphore_t usb_semaphore; // External semaphore for USB operations

/*******************************************************************************
 * Function Name: getImage
 *
 * Description:
 *   Retrieves the latest image by calling the draw function.
 *
 * Input Arguments:
 *   None
 *
 * Return Value:
 *   int8_t* - Pointer to the image buffer
 *
 *******************************************************************************/
static int8_t * getImage()
{
    return draw();
}

/*******************************************************************************
 * Function Name: cm55_inference_task
 *
 * Description:
 *   Main task for running object detection inference on the CM55 core. Initializes
 *   the object detection model, preprocesses input images, performs inference, and
 *   postprocesses results. Updates performance metrics and signals the graphics
 *   display semaphore.
 *
 * Input Arguments:
 *   void *arg - Task argument (not used)
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
void cm55_inference_task( void *arg )
{
    cy_rslt_t result;
    detection_opt_t Opt;

    // initialize object detection model inside the while loop (not globally)
    model_detail_t od_input[N_OD_INPUT];
    model_detail_t  od_output[N_OD_OUTPUT];

    ifx_object_detect_init(&model_bin, od_input, od_output);

    int    model_input_height     = od_input->dim[1];
    int    model_input_width     = od_input->dim[2];
    int    model_input_channel = od_input->dim[3];
    printf("Model input  : %d x %d x %d\r\n", model_input_width, model_input_height, model_input_channel);

    // scales input image buffer to model_input
    float scale_model_to_image = max( (float)IMAGE_WIDTH / (float)model_input_width, (float)IMAGE_HEIGHT / (float)model_input_height );
    Opt.image_width  = (float)model_input_width  * scale_model_to_image;
    Opt.image_height = (float)model_input_height * scale_model_to_image;
    Opt.threshold_IOU = Thresh_IoU;
    Opt.threshold_score = Thresh_conf;
    Opt.max_detections = Max_detections;

    while (1)
    {
        // get the latest input image to the input image buffer (image_buf_int8)
        int8_t *image_buf_int8 = getImage();
        cy_rtos_delay_milliseconds(1);

        // object detection
        volatile float time_objectDet = ifx_time_get_ms_f();

        // preprocess
        result = ifx_object_detect_preprocess(image_buf_int8, od_input);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("Error: Preprocessing finished with return code: %" PRIu32 "\r\n", result);
            continue;
        }

        // inference (on od_input)
        result = ifx_object_detect_inference(od_input->data);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("Error: Inference finished with return code: %" PRIu32 "", result);
            continue;
        }

        // posprocess
        ifx_object_detect_postprocess(&Opt, &Prediction, od_output);

        volatile float    time_end = ifx_time_get_ms_f();

        Inference_time = time_end - time_objectDet;

        // Update graphics display
        result = cy_rtos_semaphore_set(&model_semaphore);
        if (CY_RSLT_SUCCESS != result) {
            printf("\r\nModel Semphore set failed\r\n");
        }

    }
}
