/***************************************************************************//**
* \file object_detection_lib.h
*
* \brief
* This is the header file of detection utility.
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
#ifndef _OBJECT_DETECTION_H_
#define _OBJECT_DETECTION_H_

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "object_detection_structs.h"
#include "cy_utils.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "mtb_ml_utils.h"
#include "mtb_ml_common.h"
#include "mtb_ml.h"


/**
 * \brief : Initialize MTB model and link model input buffer to model input.
 *
 * \param[out]  model_input     : model input buffer
 * \param[out]  input_data      : model input data used to initialize model.
 * \param[out]  output_data     : model outnput data used to initialize model.
 *
 * \return               : MTB_ML_RESULT_SUCCESS - success
 *                       : MTB_ML_RESULT_BAD_ARG - if input parameter is invalid.
 *
 *******************************************************************************/
cy_rslt_t ifx_object_detect_init(mtb_ml_model_bin_t  *model_bin, model_detail_t *input_data, model_detail_t *output_data);


/**
 * \brief : Delete NN model runtime object and free all dynamically allocated memory. Only intended to be called once.
 *
 * \return               : MTB_ML_RESULT_SUCCESS - success
 *
 *******************************************************************************/
cy_rslt_t ifx_object_detect_deinit();


/**
 * \brief : Preprocess image buffer, updating the model_input buffer..
 *
 * \param[out]  image_buf_int8  : Data from image buffer draw.
 * \param[out]  input_data      : model input data.
 *
 * \return                      : cy_rslt_t
 *
 *******************************************************************************/
cy_rslt_t ifx_object_detect_preprocess(int8_t *image_buf_int8, model_detail_t *input_data);


/**
 * \brief : Perform NN model inference
 *
 * \param[in] object     : Pointer of model object.
 * \param[in] input      : Pointer of input data buffer
 *
 * \return               : MTB_ML_RESULT_SUCCESS - success
 *                       : MTB_ML_RESULT_BAD_ARG - if input parameter is invalid.
 *                       : MTB_ML_RESULT_INFERENCE_ERROR - if inference failure
 *
 *******************************************************************************/
cy_rslt_t ifx_object_detect_inference(int8_t *processed_data);


/**
 * \brief : Run postprocessing on the model's detections.
 *
 * \param[in]   opt                 : Detection settings struct.
 * \param[out]  prediction_OD_t     : Predictions output struct.
 * \param[out]  output_data         : Output data containing raw model output detections.
 *
 * \return                          : int32_t number of candidates after postprocessing refinement.
 *
 *******************************************************************************/
int32_t ifx_object_detect_postprocess(const detection_opt_t *opt, prediction_OD_t *predition, model_detail_t *output_data);

#endif  // _OBJECT_DETECTION_H_
