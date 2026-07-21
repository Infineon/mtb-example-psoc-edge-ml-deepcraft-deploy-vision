 /******************************************************************************
* File Name: inference_task.c
*
* Description: This file contains API calls to inference task.
*
*
********************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
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
#include <stdint.h>
#include <string.h>
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
#include "model.h"
#include "ifx_image_utils.h"
#include "lcd_task.h"
#include "inference_task.h"
#include "ifx_time_utils.h"
#include "mtb_ml_utils.h"
#include "mtb_ml_common.h"
#include "mtb_ml.h"

/*******************************************************************************
 * Global Variable
 *******************************************************************************/
/* Performance measure: Buffer wait time */
float prep_wait_buf;
/* Performance measure: YUV422 to BGR565 conversion time */
float prep_YUV422_to_bgr565;
/* Performance measure: BGR565 to display time */
float prep_bgr565_to_disp;
/* Performance measure: RGB565 to RGB888 conversion time */
float prep_RGB565_to_RGB888;

#ifdef USE_DVP_CAM
extern bool frame_ready;
#endif

__attribute__((section(".cy_socmem_data")))
/* Final output variable for object detection */
__attribute__((aligned(16))) prediction_od_t prediction;
float data_out[IMAI_DATAOUT_COUNT] = {0};
/* Inference execution time */
volatile float inference_time = 0;
/* The best class out of all i.e. the class with max value out of all classes */
float max_class_val = 0;

static model_runtime_profile_t g_model_profile = {
    .input_width = IMAGE_WIDTH,
    .input_height = IMAGE_HEIGHT,
    .max_predictions = MAX_PREDICTIONS,
    .output_columns = 0,
    .num_classes = 0,
    .has_detection_flag = false,
    .output_schema = MODEL_OUTPUT_SCHEMA_NEW,
    .preprocess_mode = MODEL_PREPROCESS_LETTERBOX,
    .bbox_mapping_mode = BBOX_MAP_LETTERBOX_TO_CAMERA
};

static bool contains_token(const char *text, const char *token)
{
    if ((text == NULL) || (token == NULL)) {
        return false;
    }

    size_t token_len = strlen(token);
    if (token_len == 0) {
        return false;
    }

    for (const char *p = text; *p != '\0'; ++p)
    {
        size_t i = 0;
        while (i < token_len)
        {
            char tc = token[i];
            char pc = p[i];
            if (pc == '\0') {
                return false;
            }
            if ((tc >= 'A') && (tc <= 'Z')) {
                tc = (char)(tc - 'A' + 'a');
            }
            if ((pc >= 'A') && (pc <= 'Z')) {
                pc = (char)(pc - 'A' + 'a');
            }
            if (pc != tc) {
                break;
            }
            ++i;
        }
        if (i == token_len) {
            return true;
        }
    }

    return false;
}

static bool detect_flag_from_label(IMAI_param_def *output_param)
{
    if ((output_param == NULL) || (output_param->shape == NULL) || (output_param->shape[1].labels == NULL)) {
        return false;
    }

    int cols = output_param->shape[1].size;
    if (cols <= 0) {
        return false;
    }

    const char *last_label = output_param->shape[1].labels[cols - 1];
    if (last_label == NULL) {
        return false;
    }

    return contains_token(last_label, "flag") || contains_token(last_label, "detect");
}

static void init_model_runtime_profile(IMAI_api_def *api_def)
{
    IMAI_param_def *input_param = &api_def->func_list[0].param_list[0];
    IMAI_param_def *output_param = &api_def->func_list[0].param_list[1];

    int in_w = IMAGE_WIDTH;
    int in_h = IMAGE_HEIGHT;
    if ((input_param->shape != NULL) && (input_param->rank >= 3))
    {
        int d0 = input_param->shape[0].size;
        int d1 = input_param->shape[1].size;
        int d2 = input_param->shape[2].size;

        if ((d0 == 3) || (d0 == 1)) {
            in_h = d1;
            in_w = d2;
        } else if ((d2 == 3) || (d2 == 1)) {
            in_h = d0;
            in_w = d1;
        } else {
            in_h = d1;
            in_w = d2;
        }
    }

    int max_predictions = output_param->shape[0].size;
    int out_cols = output_param->shape[1].size;
    int cls_new = out_cols - 4;
    int cls_legacy = out_cols - 5;

    bool has_detection_flag = detect_flag_from_label(output_param);
    int cls_count = cls_new;
    model_output_schema_t schema = MODEL_OUTPUT_SCHEMA_NEW;

    if (has_detection_flag && (cls_legacy > 0))
    {
        schema = MODEL_OUTPUT_SCHEMA_LEGACY;
        cls_count = cls_legacy;
    }
    else if (cls_new <= 0 && (cls_legacy > 0))
    {
        schema = MODEL_OUTPUT_SCHEMA_LEGACY;
        cls_count = cls_legacy;
        has_detection_flag = true;
    }

    if ((in_w <= 0) || (in_h <= 0) ||
        (in_w > MODEL_INPUT_MAX_WIDTH) || (in_h > MODEL_INPUT_MAX_HEIGHT)) {
        printf("Unsupported model input dimensions %dx%d. Max supported is %dx%d\r\n",
               in_w, in_h, MODEL_INPUT_MAX_WIDTH, MODEL_INPUT_MAX_HEIGHT);
        CY_ASSERT(0);
    }

    if ((max_predictions <= 0) || (max_predictions > MAX_PREDICTIONS) || (cls_count <= 0)) {
        printf("Unsupported model output layout (pred=%d, cols=%d, classes=%d)\r\n",
               max_predictions, out_cols, cls_count);
        CY_ASSERT(0);
    }

    g_model_profile.input_width = (uint16_t)in_w;
    g_model_profile.input_height = (uint16_t)in_h;
    g_model_profile.max_predictions = (uint16_t)max_predictions;
    g_model_profile.output_columns = (uint16_t)out_cols;
    g_model_profile.num_classes = (uint16_t)cls_count;
    g_model_profile.has_detection_flag = has_detection_flag;
    g_model_profile.output_schema = schema;
    g_model_profile.preprocess_mode = (schema == MODEL_OUTPUT_SCHEMA_LEGACY) ? MODEL_PREPROCESS_CROP : MODEL_PREPROCESS_LETTERBOX;
    g_model_profile.bbox_mapping_mode = (schema == MODEL_OUTPUT_SCHEMA_LEGACY) ? BBOX_MAP_DIRECT_MODEL : BBOX_MAP_LETTERBOX_TO_CAMERA;

    printf("Model profile: input=%ux%u pred=%u cols=%u classes=%u schema=%s preprocess=%s\r\n",
           g_model_profile.input_width,
           g_model_profile.input_height,
           g_model_profile.max_predictions,
           g_model_profile.output_columns,
           g_model_profile.num_classes,
           (g_model_profile.output_schema == MODEL_OUTPUT_SCHEMA_LEGACY) ? "legacy" : "new",
           (g_model_profile.preprocess_mode == MODEL_PREPROCESS_CROP) ? "crop" : "letterbox");
}

const model_runtime_profile_t *get_model_runtime_profile(void)
{
    return &g_model_profile;
}

/*******************************************************************************
 * Function Name: get_image
 *
 * Description:
 *   Retrieves the latest image by calling the draw function.
 *
 * Input Arguments:
 *   None
 *
 * Return Value:
 *   uint8_t* - Pointer to the image buffer
 *
 *******************************************************************************/
static uint8_t * get_image()
{
    return draw();
}

/*******************************************************************************
* Function Name: get_best_class
*******************************************************************************
*
* Summary:
*  The function calculates the best class out of all available classes.
*
* Parameters:
*  cls  - Pointer to the class array.
*  size - number of classes for the model i.e. size of class array.
*  max_cls_val - pointer to store the the best class out of all i.e.
*                the max of all classes
*
* Return:
*  max_index - Returns the idex of the best class out of all i.e.
*              the max of all classes
*
*******************************************************************************/
int8_t get_best_class(const float *cls, size_t size, float *max_cls_val) {
    if (cls == NULL || size == 0) {
    /* Array is empty */
        return -1;
    }

    int8_t max_index = 0;
    float max_value = cls[0];

    for (int8_t i = 1; i < size; i++) {
        if (cls[i] > max_value) {
            max_value = cls[i];
            max_index = i;
        }
    }

    *max_cls_val = max_value;
    return max_index;
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
    int result = IMAI_init();
    IMAI_api_def *api_def = IMAI_api();
    volatile float _time_start_prev = 0;
    (void)_time_start_prev;

    init_model_runtime_profile(api_def);

    int max_predictions = g_model_profile.max_predictions;
    /* Upper bound on class count valid for both schemas: the new schema uses
     * (output_columns - 4) classes while the legacy schema uses (output_columns - 5).
     * Sizing the scratch array to the larger (-4) value keeps it safe for either case. */
    int class_capacity = g_model_profile.output_columns - 4;
    int detection_flag_index = g_model_profile.output_columns - 1;
    float class[MAX_PREDICTIONS][(int)(class_capacity)];
    memset(class, 0, sizeof(class));

    /* Precompute letterbox parameters to reverse the transform for bbox mapping.
     * The model input is a letterboxed version of the camera frame. */
    float lb_scale_x = (float)g_model_profile.input_width  / (float)CAMERA_WIDTH;
    float lb_scale_y = (float)g_model_profile.input_height / (float)CAMERA_HEIGHT;
    float lb_scale = (lb_scale_x < lb_scale_y) ? lb_scale_x : lb_scale_y;
    int lb_new_w = (int)(CAMERA_WIDTH  * lb_scale);
    int lb_new_h = (int)(CAMERA_HEIGHT * lb_scale);
    int lb_pad_x = ((int)g_model_profile.input_width  - lb_new_w) / 2;
    int lb_pad_y = ((int)g_model_profile.input_height - lb_new_h) / 2;
    float lb_inv_scale = 1.0f / lb_scale;
    bool schema_probe_needed = !g_model_profile.has_detection_flag;

    if(result != 0)
        printf("Failed to initialize the model\r\n");

    while (true)
    {
#ifdef USE_DVP_CAM
        if (frame_ready == true)
#endif
        {
            volatile float _time_start = ifx_time_get_ms_f();
#ifdef USE_DVP_CAM
            frame_ready = false;
#endif
            /* Get the latest input image to the input image buffer. */
            uint8_t *image_buf_uint8 = get_image();
            cy_rtos_delay_milliseconds(1);

            /* Object detection */
            volatile float _time_object_det = ifx_time_get_ms_f();

            IMAI_compute(image_buf_uint8, data_out);
            /*  Bounding boxes in integer in original image size [Max_detections * 4]: [xmin, ymin, xmax, ymax]*/
            int16_t *bbox_int16 = prediction.bbox_int16;
            /*Confidence scores  */
            float *conf = prediction.conf;
            /* Class ID from the predictions*/
            uint8_t *class_id = prediction.class_id;
            int cls_count = g_model_profile.num_classes;

            int32_t actual_nr_predictions = 0;

            if (schema_probe_needed && (g_model_profile.output_columns > 5))
            {
                int one_count = 0;
                int zero_count = 0;
                int invalid_count = 0;
                for (int r = 0; r < max_predictions; r++)
                {
                    float v = data_out[detection_flag_index * max_predictions + r];
                    if ((v > 0.8f) && (v < 1.2f)) {
                        one_count++;
                    } else if ((v >= -0.2f) && (v < 0.2f)) {
                        zero_count++;
                    } else {
                        invalid_count++;
                    }
                }

                if ((one_count > 0) && (invalid_count == 0))
                {
                    g_model_profile.has_detection_flag = true;
                    g_model_profile.output_schema = MODEL_OUTPUT_SCHEMA_LEGACY;
                    g_model_profile.preprocess_mode = MODEL_PREPROCESS_CROP;
                    g_model_profile.bbox_mapping_mode = BBOX_MAP_DIRECT_MODEL;
                    g_model_profile.num_classes = g_model_profile.output_columns - 5;
                    printf("Runtime schema probe selected legacy detection-flag mode (ones=%d zeros=%d)\r\n", one_count, zero_count);
                }
                schema_probe_needed = false;
                cls_count = g_model_profile.num_classes;
            }

            for (int r = 0; r < max_predictions; r++) {
                if (g_model_profile.has_detection_flag)
                {
                    if (data_out[detection_flag_index * max_predictions + r] <= 0.0f) {
                        break;
                    }
                }

                /* Single pass: copy class scores, find best class index and value */
                float best_score = 0.0f;
                uint8_t best_idx = 0;
                for (int c = 0; c < cls_count; c++)
                {
                    float s = data_out[(c + 4) * max_predictions + r];
                    class[r][c] = s;
                    if (s > best_score)
                    {
                        best_score = s;
                        best_idx = (uint8_t)c;
                    }
                }
                if ((!g_model_profile.has_detection_flag) && (best_score == 0.0f))
                    break;

                actual_nr_predictions++;

                class_id[r] = best_idx;
                conf[r] = best_score;
                memset(prediction.class_string[r], 0, MAX_CLASS_LEN);
                if ((api_def->func_list[0].param_list[1].shape[1].labels != NULL) &&
                    (api_def->func_list[0].param_list[1].shape[1].labels[best_idx + 4] != NULL))
                {
                    strncpy(prediction.class_string[r],
                            api_def->func_list[0].param_list[1].shape[1].labels[best_idx + 4],
                            MAX_CLASS_LEN - 1);
                }
                else
                {
                    strncpy(prediction.class_string[r], "class", MAX_CLASS_LEN - 1);
                }

                /* Buffer layout is attribute-major: [x0..x4, y0..y4, w0..w4, h0..h4, cls0_0..cls0_4, ...] */
                float x = data_out[0 * max_predictions + r];
                float y = data_out[1 * max_predictions + r];
                float w = data_out[2 * max_predictions + r];
                float h = data_out[3 * max_predictions + r];

                /* Reverse the letterbox transform: model pixels → camera pixels */
                float cam_cx;
                float cam_cy;
                float cam_w;
                float cam_h;

                if (g_model_profile.bbox_mapping_mode == BBOX_MAP_DIRECT_MODEL)
                {
                    cam_cx = x * CAMERA_WIDTH;
                    cam_cy = y * CAMERA_HEIGHT;
                    cam_w = w * CAMERA_WIDTH;
                    cam_h = h * CAMERA_HEIGHT;
                }
                else
                {
                    float mx = x * g_model_profile.input_width;
                    float my = y * g_model_profile.input_height;
                    float mw = w * g_model_profile.input_width;
                    float mh = h * g_model_profile.input_height;

                    cam_cx = (mx - lb_pad_x) * lb_inv_scale;
                    cam_cy = (my - lb_pad_y) * lb_inv_scale;
                    cam_w  = mw * lb_inv_scale;
                    cam_h  = mh * lb_inv_scale;
                }

                *bbox_int16++ = (int16_t)(cam_cx - HALF(cam_w) + RND_F2I_FACTOR);
                *bbox_int16++ = (int16_t)(cam_cy - HALF(cam_h) + RND_F2I_FACTOR);
                *bbox_int16++ = (int16_t)(cam_cx + HALF(cam_w) + RND_F2I_FACTOR);
                *bbox_int16++ = (int16_t)(cam_cy + HALF(cam_h) + RND_F2I_FACTOR);

            }

            prediction.count = actual_nr_predictions;
            volatile float _time_end = ifx_time_get_ms_f();

            inference_time = _time_end - _time_object_det;

            result = cy_rtos_semaphore_set(&model_semaphore);
            if (CY_RSLT_SUCCESS != result) {
                printf("\r\nModel Semphore set failed\r\n");
            }

            _time_start_prev = _time_start;
        }
    }
}
