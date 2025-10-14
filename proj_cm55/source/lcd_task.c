/******************************************************************************
* File Name:   lcd_task.c
*
* Description: This file implements the LCD display modules.
*
* Related Document: See README.md
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
#include "cybsp.h"
#include "retarget_io_init.h"
#include "no_camera_img.h"
#include "camera_not_supported_img.h"
#include "mtb_disp_dsi_waveshare_4p3.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "definitions.h"
#include "usb_camera_task.h"
#include "lcd_task.h"
#include "inference_task.h"
#include "ifx_gui_render.h"
#include "font_16x36.h"
#include "ifx_image_utils.h"
#include "ifx_time_utils.h"
#include "object_detection_structs.h"
#include "model_info.h" /*import CLASS_STRING_LIST and its values*/ 
#include "lcd_graphics.h"

#if defined(MTB_SHARED_MEM)
#include "shared_mem.h"
#endif

/*******************************************************************************
 * Macros
 *******************************************************************************/
#define GFX_TASK_DELAY_MS                   (30U)
#define I2C_CONTROLLER_IRQ_PRIORITY         (4U)

#define GPU_INT_PRIORITY                    (3U)
#define DC_INT_PRIORITY                     (3U)

#define COLOR_DEPTH                         (16U)
#define BITS_PER_PIXEL                      (8U)

#define DISPLAY_HEIGHT                      (480U)
#define DISPLAY_WIDTH                       (832U)

#define DEFAULT_GPU_CMD_BUFFER_SIZE         ((64U) * (1024U))
#define GPU_TESSELLATION_BUFFER_SIZE        ((DISPLAY_HEIGHT) * 128U)

#define FRAME_BUFFER_SIZE                   ((DISPLAY_WIDTH) * (DISPLAY_HEIGHT) * ((COLOR_DEPTH) / (BITS_PER_PIXEL)))

#define VGLITE_HEAP_SIZE                    (((FRAME_BUFFER_SIZE) * (3)) + \
                                             (((DEFAULT_GPU_CMD_BUFFER_SIZE) + (GPU_TESSELLATION_BUFFER_SIZE)) * (NUM_IMAGE_BUFFERS)) + \
                                             ((CAMERA_BUFFER_SIZE) * (NUM_IMAGE_BUFFERS + 1)))

#define GPU_MEM_BASE                        (0x0U)
#define TICK_VAL                            (1U)

#define WHITE_COLOR                         (0x00FFFFFFU)
#define BLACK_COLOR                         (0x00000000U)
#define TARGET_NUM_FRAMES                   (15U)

#define RESET_VAL                           (0U)

#define OUTER_BOX_X_POS                     (0)
#define OUTER_BOX_Y_POS                     (0)
#define OUTER_BOX_HIGHT                     (512)
#define OUTER_BOX_WIDTH                     (512)

#define BOX_X_POS                           (0)
#define BOX_Y_POS                           (0)
#define BOX_HIGHT                           (100)
#define BOX_WIDTH                           (100)

/* Display I2C controller */
#ifdef USE_KIT_PSE84_AI
#define DISPLAY_I2C_CONTROLLER_HW     CYBSP_I2C_DISPLAY_CONTROLLER_HW
#define DISPLAY_I2C_CONTROLLER_IRQ    CYBSP_I2C_DISPLAY_CONTROLLER_IRQ
#define DISPLAY_I2C_CONTROLLER_config CYBSP_I2C_DISPLAY_CONTROLLER_config
#else
#define DISPLAY_I2C_CONTROLLER_HW     CYBSP_I2C_CONTROLLER_HW
#define DISPLAY_I2C_CONTROLLER_IRQ    CYBSP_I2C_CONTROLLER_IRQ
#define DISPLAY_I2C_CONTROLLER_config CYBSP_I2C_CONTROLLER_config
#endif

#define NO_CAMERA_IMG_X_POS                 ((MTB_DISP_WAVESHARE_4P3_HOR_RES / 2U) \
                                             - ((NO_CAMERA_IMG_WIDTH / 2U) + 10))
#define NO_CAMERA_IMG_Y_POS                 ((MTB_DISP_WAVESHARE_4P3_VER_RES / 2U) \
                                             - (NO_CAMERA_IMG_HEIGHT / 2U))

#define CAMERA_NOT_SUPPORTED_IMG_X_POS      ((MTB_DISP_WAVESHARE_4P3_HOR_RES / 2U) \
                                             - ((CAMERA_NOT_SUPPORTED_IMG_WIDTH / 2U) + 10))
#define CAMERA_NOT_SUPPORTED_IMG_Y_POS      ((MTB_DISP_WAVESHARE_4P3_VER_RES / 2U) \
                                             - (CAMERA_NOT_SUPPORTED_IMG_HEIGHT / 2U))

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
static float last_successful_frame_time = 0;                     // Last successful USB frame time
static int recovery_attempts = 0;                               // USB recovery attempt counter
extern cy_semaphore_t usb_semaphore;                            // USB semaphore for synchronization
extern prediction_OD_t Prediction;                              // Object detection prediction
extern volatile float Inference_time;                           // Inference time for model
extern cy_semaphore_t model_semaphore;                          // Model semaphore for synchronization
extern uint8_t _device_connected;                               // Device connected
volatile float time_start1;                                     // Start time
cy_stc_gfx_context_t gfx_context;                               // Graphics context structure
vg_lite_buffer_t *renderTarget;                                 // Render target buffer
vg_lite_buffer_t usb_yuv_frames[NUM_IMAGE_BUFFERS];             // USB YUV frame buffers
vg_lite_buffer_t bgr565;                                        // BGR565 buffer
vg_lite_buffer_t display_buffer[3];                             // Double display frame buffers
float scale_Cam2Disp;                                           // Scale factor from camera to display

CY_SECTION(".cy_socmem_data")
__attribute__((aligned(64)))
int8_t bgr888_int8[(IMAGE_HEIGHT) * (IMAGE_WIDTH) * 3] = {0};   // BGR888 integer buffer

CY_SECTION(".cy_xip") __attribute__((used))
uint8_t contiguous_mem[VGLITE_HEAP_SIZE];                       // Contiguous memory for VGLite heap
volatile void *vglite_heap_base = &contiguous_mem;              // VGLite heap base address
volatile bool fb_pending = false;                               // Framebuffer pending flag
volatile bool button_debouncing = false;                        // Flag indicating if button debouncing is active
volatile uint32_t button_debounce_timestamp = 0;                // Timestamp for button debouncing in milliseconds

/*******************************************************************************
 * Global Variables - Shared memory variable
 *******************************************************************************/
#if defined(MTB_SHARED_MEM)
oob_shared_data_t oob_shared_data_ns;
#endif

/*******************************************************************************
 * Global Variables - I2C Controller Configuration
 *******************************************************************************/
cy_stc_scb_i2c_context_t i2c_controller_context;                // I2C controller context

/*******************************************************************************
 * Global Variables - Interrupt Configurations
 *******************************************************************************/
cy_stc_sysint_t dc_irq_cfg =                                   // DC Interrupt Configuration
{
    .intrSrc      = gfxss_interrupt_dc_IRQn,                   // DC interrupt source
    .intrPriority = DC_INT_PRIORITY                            // DC interrupt priority
};

cy_stc_sysint_t gpu_irq_cfg =                                  // GPU Interrupt Configuration
{
    .intrSrc      = gfxss_interrupt_gpu_IRQn,                  // GPU interrupt source
    .intrPriority = GPU_INT_PRIORITY                           // GPU interrupt priority
};

cy_stc_sysint_t i2c_controller_irq_cfg =                       // I2C Controller Interrupt Configuration
{
    .intrSrc      = DISPLAY_I2C_CONTROLLER_IRQ,                  // I2C controller interrupt source
    .intrPriority = I2C_CONTROLLER_IRQ_PRIORITY                // I2C controller interrupt priority
};

/*******************************************************************************
 * Local Variables
 *******************************************************************************/
static GFXSS_Type *base = (GFXSS_Type *)GFXSS;                 // Graphics subsystem base address
static vg_lite_matrix_t matrix;                                // VGLite transformation matrix
static int display_offset_x = 0;                               // Display X offset
static int display_offset_y = 0;                               // Display Y offset

static int16_t pathData[] = {
    2, 100, 100,                                               // Move to (minX, minY)
    4, 300, 100,                                               // Line from (minX, minY) to (maxX, minY)
    4, 300, 200,                                               // Line from (maxX, minY) to (maxX, maxY)
    4, 100, 200,                                               // Line from (maxX, maxY) to (minX, maxY)
    4, 100, 100,                                               // Line from (minX, maxY) to (minX, minY)
    0,
};

static vg_lite_path_t path = {
    .bounding_box = {0, 0, CAMERA_WIDTH, CAMERA_HEIGHT},        // left, top, right, bottom
    .quality = VG_LITE_HIGH,                                   // Quality
    .format = VG_LITE_S16,                                     // Format
    .uploaded = {0},                                           // Uploaded
    .path_length = sizeof(pathData),                           // Path length
    .path = pathData,                                          // Path data
    .path_changed = 1                                          // Path changed
};

static uint8_t color_r[4] = {0, 0, 227, 8};                    // Red components: {Green, Black, Red, Blue}
static uint8_t color_g[4] = {255, 0, 66, 24};                  // Green components: {Green, Black, Red, Blue}
static uint8_t color_b[4] = {0, 0, 52, 168};                   // Blue components: {Green, Black, Red, Blue}

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888_i8(uint8_t *src_bgr565, int width, int height,
                                        int8_t *dst_rgb888_i8, int dst_width, int dst_height);
uint32_t ifx_lcd_get_Display_Width(void);
uint32_t ifx_lcd_get_Display_Height(void);
void ifx_lcd_display_Rect(uint16_t x0, uint16_t y0, uint8_t *image, uint16_t width, uint16_t height);


CY_SECTION_ITCM_BEGIN
/*******************************************************************************
* Function Name: mirrorImage
********************************************************************************
* Description: Mirrors an image horizontally by swapping pixels from left to right
*              in the provided buffer. The function assumes a fixed bytes-per-pixel
*              value of 2 (e.g., for RGB565 format) and operates on a buffer with
*              dimensions defined by CAMERA_WIDTH and CAMERA_HEIGHT.
* Parameters:
*   - buffer: Pointer to the vg_lite_buffer_t structure containing the image data
*
* Return:
*   None
********************************************************************************/
void mirrorImage(vg_lite_buffer_t *buffer) {
    uint8_t temp[4];
    uint8_t *start, *end;
    int m, n;
    int bytes_per_pixel  = 0 ;

    bytes_per_pixel = 2;

    for (m = 0; m < CAMERA_HEIGHT ; m++) {

        start = buffer->memory + m * (CAMERA_WIDTH * bytes_per_pixel);

        end = start + (CAMERA_WIDTH - 1) * bytes_per_pixel;

        for (n = 0; n < CAMERA_WIDTH / 2; n++) {

            for (int i = 0; i < bytes_per_pixel; i++) {
                temp[i] = start[i];
            }

            for (int i = 0; i < bytes_per_pixel; i++) {
                start[i] = end[i];
            }

            for (int i = 0; i < bytes_per_pixel; i++) {
                end[i] = temp[i];
            }

            start += bytes_per_pixel;
            end -= bytes_per_pixel;
        }
    }
}

CY_SECTION_ITCM_END
/*******************************************************************************
* Function Name: cleanup
********************************************************************************
* Description: Deallocates resources and frees memory used by the VGLite
*              graphics library. This function should be called to ensure proper
*              cleanup of VGLite resources when they are no longer needed.
* Parameters:
*   None
*
* Return:
*   None
********************************************************************************/
static void cleanup ( void )
{
    /* Deallocate all the resource and free up all the memory */
    vg_lite_close();
}


CY_SECTION_ITCM_BEGIN
/*******************************************************************************
* Function Name: draw
********************************************************************************
 * Description: Processes and renders an image from a USB camera buffer. The function
 *              performs the following steps:
 *              1. Waits for a ready image buffer from the camera.
 *              2. Converts a 320x240 YUYV 422 image to 320x240 BGR565 format.
 *              3. Optionally mirrors the image if the 3MP camera is disabled.
 *              4. Scales the 320x240 BGR565 image to 800x600 BGR565 for display.
 *              5. Converts the 320x240 BGR565 image to 256x240 BGR888 format (offset by -128).
 *              6. Tracks performance metrics for each step and returns the BGR888 buffer.
 *              The function handles errors by invoking cleanup and asserting on failure.
* Parameters:
*   bgr888_int8 Pointer to the int8_t BGR888 buffer containing the processed image data
*
* Return:
*   None
*
********************************************************************************/
int8_t * draw(void)
{
    vg_lite_error_t error;      // = VG_LITE_SUCCESS;
    volatile uint32_t time_draw_start = ifx_time_get_ms_f();

    extern uint8_t lastBuffer;
    extern VideoBuffer_t _ImageBuff[];

    // find a ready buffer
    uint32_t workBuffer = lastBuffer;

    while (_ImageBuff[workBuffer].BufReady == 0) {
        for (int32_t ii = 0; ii < NUM_IMAGE_BUFFERS; ii++)
            if (_ImageBuff[(workBuffer + ii) % NUM_IMAGE_BUFFERS].BufReady == 1) {
                workBuffer = (workBuffer + ii) % NUM_IMAGE_BUFFERS;
                break;
            }
        cy_rtos_delay_milliseconds(1);
    }

    // reset all other buffers to available for input from camera
    for (int32_t ii = 1; ii < NUM_IMAGE_BUFFERS; ii++)
        _ImageBuff[(workBuffer + ii) % NUM_IMAGE_BUFFERS].BufReady = 0;

    /* convert 320x240 YUYV 422 image into 320*240 BGR565 (scale:1) */
    volatile uint32_t time_draw_1 = ifx_time_get_ms_f();
    error = vg_lite_blit(&bgr565, &usb_yuv_frames[workBuffer],
                         NULL,                       // identity matrix
                         VG_LITE_BLEND_NONE,
                         0,
                         VG_LITE_FILTER_POINT);
    if (error) {
        printf("\r\nvg_lite_blit() (320x240 YUYV 422 ==> 320*240 BGR565) returned error %d\r\n", error);
        cleanup();
        CY_ASSERT(0);
    }

    vg_lite_finish();

    if (!point3mp_camera_enabled) {
        mirrorImage(&bgr565);
    }

    /* convert 320x240 BGR565 image into 800x600 BGR565 (scale:2.5) */
    volatile uint32_t time_draw_3 = ifx_time_get_ms_f();
    error = vg_lite_blit(renderTarget, &bgr565,
                         &matrix,
                         VG_LITE_BLEND_NONE,
                         0,
                         VG_LITE_FILTER_POINT);
    if (error) {
        printf("\r\nvg_lite_blit() (320x240 BGR565 ==> 800x600 display BGR565) returned error %d\r\n", error);
        cleanup();
        CY_ASSERT(0);
    }
    vg_lite_finish();

    /* Clear USB buffer */
    _ImageBuff[workBuffer].NumBytes = 0;
    _ImageBuff[workBuffer].BufReady = 0;

    /* Convert 320x240 BGR565 image into 256x240 BGR888 - 128 */
    volatile uint32_t time_draw_5 = ifx_time_get_ms_f();
    ifx_image_conv_RGB565_to_RGB888_i8(bgr565.memory, CAMERA_WIDTH, CAMERA_HEIGHT, bgr888_int8, IMAGE_WIDTH, IMAGE_HEIGHT);

    volatile uint32_t time_draw_end = ifx_time_get_ms_f();
    // performance measures: time
    extern float Prep_Wait_Buf, Prep_YUV422_to_bgr565, Prep_bgr565_to_Disp, Prep_RGB565_to_RGB888;
    Prep_Wait_Buf         = time_draw_1 - time_draw_start;
    Prep_YUV422_to_bgr565 = time_draw_3 - time_draw_1;
    Prep_bgr565_to_Disp   = time_draw_5 - time_draw_3;
    Prep_RGB565_to_RGB888 = time_draw_end - time_draw_5;

    return bgr888_int8;
}
CY_SECTION_ITCM_END

/*******************************************************************************
* Function Name: dc_irq_handler
********************************************************************************
* Summary: This is the display controller I2C interrupt handler.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void dc_irq_handler ( void )
{
    fb_pending = false;
    Cy_GFXSS_Clear_DC_Interrupt(base, &gfx_context);
}

/*******************************************************************************
* Function Name: dc_gpu_irq_handlerirq_handler
********************************************************************************
* Summary: This is the GPU interrupt handler.
*
* Parameters:
*   None
*
* Return:
*   None
*
*******************************************************************************/
static void gpu_irq_handler ( void )
{
    Cy_GFXSS_Clear_GPU_Interrupt(base, &gfx_context);
    vg_lite_IRQHandler();
}

/*******************************************************************************
* Function Name: draw_rectangle
*******************************************************************************
*
* Summary:
*  Draws a rectangle on the render target using the VGLite graphics library.
*
* Parameters:
*  color  - The color of the rectangle in 32-bit format.
*  min_x  - The x-coordinate of the top-left corner of the rectangle.
*  min_y  - The y-coordinate of the top-left corner of the rectangle.
*  max_x  - The x-coordinate of the bottom-right corner of the rectangle.
*  max_y  - The y-coordinate of the bottom-right corner of the rectangle.
*
* Return:
*  None
*
*
*******************************************************************************/
void draw_rectangle(uint32_t color, int16_t min_x, int16_t min_y, int16_t max_x, int16_t max_y)
{
    pathData[1]  = min_x;  pathData[2]  = min_y;  // Top-left corner
    pathData[4]  = max_x;  pathData[5]  = min_y;  // Top-right corner
    pathData[7]  = max_x;  pathData[8]  = max_y;  // Bottom-right corner
    pathData[10] = min_x;  pathData[11] = max_y;  // Bottom-left corner
    pathData[13] = min_x;  pathData[14] = min_y;  // Close path back to top-left
    vg_lite_draw(renderTarget, &path, VG_LITE_FILL_EVEN_ODD, &matrix, VG_LITE_BLEND_SRC_OVER, color);
}

/*******************************************************************************
* Function Name: update_box_data
*******************************************************************************
*
* Summary:
*  Updates and draws bounding boxes on the render target based on object detection
*  predictions. Scales bounding box coordinates to display dimensions, sets colors
*  based on class IDs, and draws rectangles and labels for each detected object.
*
* Parameters:
*  renderTarget - Pointer to the vg_lite_buffer_t structure for rendering.
*  prediction   - Pointer to the prediction_OD_t structure containing object
*                 detection results, including bounding box coordinates, class IDs,
*                 and confidence scores.
*
* Return:
*  None
*
*
*******************************************************************************/
void update_box_data(vg_lite_buffer_t *renderTarget, prediction_OD_t *prediction)
{
    for (int32_t i = 0; i < prediction->count; i++) {
        int32_t jj = i << 2; // Index for bounding box coordinates
        int32_t id = prediction->class_id[i];
        int32_t cid = (id >= 0) ? (id % 7) + 1 : 0; // Map class ID to color index

        // Scale and offset bounding box coordinates
        uint32_t xmin = (uint32_t)(prediction->bbox_int16[jj] * scale_Cam2Disp) + display_offset_x;
        uint32_t ymin = (uint32_t)(prediction->bbox_int16[jj + 1] * scale_Cam2Disp) + display_offset_y;
        uint32_t xmax = (uint32_t)(prediction->bbox_int16[jj + 2] * scale_Cam2Disp) + display_offset_x;
        uint32_t ymax = (uint32_t)(prediction->bbox_int16[jj + 3] * scale_Cam2Disp) + display_offset_y;

        // Set foreground color based on class ID
        if (cid == 3) {
            ifx_lcd_set_FGcolor(8, 24, 168); // Blue
        } else if (cid == 1) {
            ifx_lcd_set_FGcolor(0, 0, 0); // Black
        } else if (cid == 2) {
            ifx_lcd_set_FGcolor(227, 66, 24); // Red
        }

        // Draw rectangle for the bounding box
        ifx_lcd_draw_Rect(xmin, ymin, xmax, ymax);

        // Draw label with class name and confidence score for valid class IDs
        if (id >= 0 && cid < 4) {
            ifx_set_bg_color((color_r[cid] << 16) | (color_g[cid] << 8) | color_b[cid]);
            ifx_print_to_buffer(xmin + 8, ymin - 36, "%s, %.2f", CLASS_STRING_LIST[id], prediction->conf[i] * 100.0f);
        }

        // Render the buffer
        ifx_draw_buffer(renderTarget->memory);
    }
}

/*******************************************************************************
* Function Name: update_box_data1
*******************************************************************************
*
* Summary:
*  Displays model inference time on the render target at a fixed position. Sets
*  the background color using predefined RGB values and prints the inference time
*  with a label.
*
* Parameters:
*  renderTarget - Pointer to the vg_lite_buffer_t structure for rendering.
*  num          - Floating-point value representing the model inference time in
*                 milliseconds.
*
* Return:
*  None
*
*
*******************************************************************************/
void update_box_data1(vg_lite_buffer_t *renderTarget, float num)
{
    ifx_set_bg_color((color_r[3] << 16) | (color_g[3] << 8) | color_b[3]); // Set background color
    ifx_print_to_buffer(10, 440, "%s %.1f %s", "Model", num, "ms");         // Print inference time
    ifx_draw_buffer(renderTarget->memory);                                   // Render the buffer
}


/*******************************************************************************
* Function Name: VG_LITE_ERROR_EXIT
*******************************************************************************
*
* Summary:
*  Handles VGLite error conditions by printing an error message with the status
*  code, calling the cleanup function to deallocate resources, and triggering an
*  assertion to halt execution.
*
* Parameters:
*  msg           - Pointer to a character string describing the error context.
*  vglite_status - VGLite error code indicating the specific error condition.
*
* Return:
*  None
*
*
*******************************************************************************/
void VG_LITE_ERROR_EXIT(char *msg, vg_lite_error_t vglite_status)
{
    printf("%s %d\r\n", msg, vglite_status); // Print error message and status
    cleanup();                               // Deallocate VGLite resources
    CY_ASSERT(0);                            // Halt execution
}

/*******************************************************************************
* Function Name: disp_i2c_controller_interrupt
********************************************************************************
* Summary:
*  I2C controller ISR which invokes Cy_SCB_I2C_Interrupt to perform I2C transfer
*  as controller.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void i2c_controller_interrupt(void)
{
    Cy_SCB_I2C_Interrupt(DISPLAY_I2C_CONTROLLER_HW, &i2c_controller_context);
}

/*******************************************************************************
* Function Name: VG_switch_frame
*******************************************************************************
*
* Summary:
*  Switches the video/graphics layer frame buffer and manages buffer swapping for
*  display rendering. Signals the USB semaphore based on camera enable status.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
void VG_switch_frame(void)
{
    static int current_buffer = 0;

    /* Set Video/Graphics layer buffer address and transfer the frame buffer to DC */
    Cy_GFXSS_Set_FrameBuffer(base, (uint32_t*)renderTarget->address, &gfx_context);
    __DMB();

    /* Swap buffers */
    current_buffer ^= 1;
    renderTarget = &display_buffer[current_buffer];

    __DMB();

    if (!Logitech_camera_enabled)
    {
        cy_rtos_semaphore_set(&usb_semaphore);
    }
    else
    {
        cy_rslt_t result = cy_rtos_semaphore_get(&usb_semaphore, 0xFFFFFFFF);
        if (CY_RSLT_SUCCESS != result)
        {
            printf("[USB Camera] USB Semaphore set failed\r\n");
        }
    }
}

/*******************************************************************************
* Function Name: init_buffer_system
*******************************************************************************
*
* Summary:
*  Initializes the buffer system by resetting image buffer states, clearing buffer
*  counters, and resetting timeout tracking variables.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
static void init_buffer_system(void)
{
    // Ensure clean startup state
    for (int i = 0; i < NUM_IMAGE_BUFFERS; i++)
    {
        _ImageBuff[i].BufReady = 0;
        _ImageBuff[i].NumBytes = 0;
    }
    __DMB();

    lastBuffer = 0;
    __DMB();

    // Reset timeout tracking
    last_successful_frame_time = 0;
    recovery_attempts = 0;
}


/*******************************************************************************
* Function Name: cm55_ns_gfx_task
*******************************************************************************
*
* Summary:
*  Initializes and manages the graphics subsystem, including display controller,
*  I2C, VGLite, and buffer systems. Handles rendering of camera frames or fallback
*  images based on device connection status, updates display with bounding box data,
*  and manages frame buffer swapping.
*
* Parameters:
*  arg - Unused task argument
*
* Return:
*  None
*
*******************************************************************************/
void cm55_ns_gfx_task(void *arg)
{
    CY_UNUSED_PARAMETER(arg);
    vg_lite_error_t vglite_status = VG_LITE_SUCCESS;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_en_gfx_status_t status = CY_GFX_SUCCESS;
    cy_en_sysint_status_t sysint_status = CY_SYSINT_SUCCESS;
    cy_en_scb_i2c_status_t i2c_result = CY_SCB_I2C_SUCCESS;

    /* Set frame buffer address to the GFXSS configuration structure */
    GFXSS_config.dc_cfg->gfx_layer_config->buffer_address = (gctADDRESS *)vglite_heap_base;
    GFXSS_config.dc_cfg->gfx_layer_config->uv_buffer_address = (gctADDRESS *)vglite_heap_base;

    /* Initialize the graphics subsystem according to the configuration */
    status = Cy_GFXSS_Init(base, &GFXSS_config, &gfx_context);
    if (CY_GFX_SUCCESS != status)
    {
        printf("Graphics subsystem initialization failed: Cy_GFXSS_Init() returned error %d\r\n", status);
        CY_ASSERT(0);
    }

    mtb_hal_syspm_lock_deepsleep();

    /* Setup Display Controller interrupt */
    sysint_status = Cy_SysInt_Init(&dc_irq_cfg, dc_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        printf("Error in registering DC interrupt: %d\r\n", sysint_status);
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(GFXSS_DC_IRQ);
    Cy_GFXSS_Clear_DC_Interrupt(base, &gfx_context);

    /* Initialize GFX GPU interrupt */
    sysint_status = Cy_SysInt_Init(&gpu_irq_cfg, gpu_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        printf("Error in registering GPU interrupt: %d\r\n", sysint_status);
        CY_ASSERT(0);
    }
    Cy_GFXSS_Enable_GPU_Interrupt(base);
    NVIC_EnableIRQ(GFXSS_GPU_IRQ);

    /* Initialize the I2C in controller mode */
    i2c_result = Cy_SCB_I2C_Init(DISPLAY_I2C_CONTROLLER_HW, &DISPLAY_I2C_CONTROLLER_config, &i2c_controller_context);
    if (CY_SCB_I2C_SUCCESS != i2c_result)
    {
        printf("I2C controller initialization failed !!\n");
        CY_ASSERT(0);
    }

    /* Initialize the I2C interrupt */
    sysint_status = Cy_SysInt_Init(&i2c_controller_irq_cfg, &i2c_controller_interrupt);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        printf("I2C controller interrupt initialization failed\r\n");
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ(i2c_controller_irq_cfg.intrSrc);
    Cy_SCB_I2C_Enable(DISPLAY_I2C_CONTROLLER_HW);

    /* Initialize Waveshare 4.3-Inch display */
    i2c_result = mtb_disp_waveshare_4p3_init(DISPLAY_I2C_CONTROLLER_HW, &i2c_controller_context);
    if (CY_SCB_I2C_SUCCESS != i2c_result)
    {
        printf("Waveshare 4.3-Inch display init failed with status = %u\r\n", (unsigned int)i2c_result);
        CY_ASSERT(0);
    }

    /* Initialize VGLite memory parameters */
    vg_module_parameters_t vg_params;
    vg_params.register_mem_base = (uint32_t)GFXSS_GFXSS_GPU_GCNANO;
    vg_params.gpu_mem_base[0] = GPU_MEM_BASE;
    vg_params.contiguous_mem_base[0] = (volatile void *)vglite_heap_base;
    vg_params.contiguous_mem_size[0] = VGLITE_HEAP_SIZE;

    /* Initialize VGLite */
    vg_lite_init_mem(&vg_params);
    vglite_status = vg_lite_init(DISPLAY_WIDTH, DISPLAY_HEIGHT);
    if (VG_LITE_SUCCESS != vglite_status)
    {
        VG_LITE_ERROR_EXIT("vg_lite engine init failed: vg_lite_init() returned error %d\r\n", vglite_status);
    }

    /* Setup double display frame buffers */
    for (int32_t ii = 0; ii < 2; ii++)
    {
        display_buffer[ii].width = DISPLAY_WIDTH;
        display_buffer[ii].height = DISPLAY_HEIGHT;
        display_buffer[ii].format = VG_LITE_BGR565;
        vglite_status = vg_lite_allocate(&display_buffer[ii]);
        if (VG_LITE_SUCCESS != vglite_status)
        {
            VG_LITE_ERROR_EXIT("display_buffer[] allocation failed in vglite space: vg_lite_allocate() returned error %d\r\n", vglite_status);
        }
    }
    renderTarget = &display_buffer[0];

    /* Allocate the camera buffers */
    for (int32_t i = 0; i < NUM_IMAGE_BUFFERS; i++)
    {
        usb_yuv_frames[i].width = CAMERA_WIDTH;
        usb_yuv_frames[i].height = CAMERA_HEIGHT;
        usb_yuv_frames[i].format = VG_LITE_YUYV;
        usb_yuv_frames[i].image_mode = VG_LITE_NORMAL_IMAGE_MODE;
        vglite_status = vg_lite_allocate(&usb_yuv_frames[i]);
        if (VG_LITE_SUCCESS != vglite_status)
        {
            VG_LITE_ERROR_EXIT("camera buffers allocation failed in vglite space: vg_lite_allocate() returned error %d\r\n", vglite_status);
        }
    }

    /* Allocate the work camera buffer */
    bgr565.width = CAMERA_WIDTH;
    bgr565.height = CAMERA_HEIGHT;
    bgr565.format = VG_LITE_BGR565;
    bgr565.image_mode = VG_LITE_NORMAL_IMAGE_MODE;
    vglite_status = vg_lite_allocate(&bgr565);
    if (VG_LITE_SUCCESS != vglite_status)
    {
        VG_LITE_ERROR_EXIT("work camera image bgr565 allocation failed in vglite space: vg_lite_allocate() returned error %d\r\n", vglite_status);
    }

    /* Clear the buffer with black color */
    vglite_status = vg_lite_clear(renderTarget, NULL, BLACK_COLOR);
    if (VG_LITE_SUCCESS != vglite_status)
    {
        VG_LITE_ERROR_EXIT("Clear failed: vg_lite_clear() returned error %d\r\n", vglite_status);
    }

    /* Define the transformation matrix for camera image to display */
    vg_lite_identity(&matrix);
    float scale_Cam2Disp_x = (float)(DISPLAY_WIDTH) / (float)CAMERA_WIDTH;
    float scale_Cam2Disp_y = (float)(DISPLAY_HEIGHT) / (float)CAMERA_HEIGHT;
    scale_Cam2Disp = max(scale_Cam2Disp_x, scale_Cam2Disp_y);
    vg_lite_scale(scale_Cam2Disp, scale_Cam2Disp, &matrix);

    /* Move the scaled frame to the display center */
    float translate_x = ((DISPLAY_WIDTH) / scale_Cam2Disp - CAMERA_WIDTH) * 0.5f;
    float translate_y = (DISPLAY_HEIGHT / scale_Cam2Disp - CAMERA_HEIGHT) * 0.5f;
    vg_lite_translate(translate_x, translate_y, &matrix);

    display_offset_x = ((DISPLAY_WIDTH) - scale_Cam2Disp * IMAGE_WIDTH) / 2;
    display_offset_y = (DISPLAY_HEIGHT - scale_Cam2Disp * CAMERA_HEIGHT) / 2;

    Cy_GFXSS_Set_FrameBuffer(base, (uint32_t *)renderTarget->address, &gfx_context);
    vg_lite_flush();
    Cy_GFXSS_Set_FrameBuffer(base, (uint32_t *)renderTarget->address, &gfx_context);

    /* Delay for USB enumeration to complete before rendering */
    vTaskDelay(pdMS_TO_TICKS(1500));

    /* Initialize buffer states - clear any startup artifacts */
    for (int i = 0; i < NUM_IMAGE_BUFFERS; i++)
    {
        _ImageBuff[i].BufReady = 0;
        _ImageBuff[i].NumBytes = 0;
    }
    __DMB();

    init_buffer_system();

    for (;;)
    {

        if (!_device_connected)
        {
            int i = 0;
            while (i < 2)
            {
                i++;
                __DMB();
                vg_lite_finish();
                Cy_GFXSS_Set_FrameBuffer(base, (uint32_t *)renderTarget->address, &gfx_context);
                __DMB();

                /* Clear the buffer with black color */
                vglite_status = vg_lite_clear(renderTarget, NULL, BLACK_COLOR);
                if (VG_LITE_SUCCESS != vglite_status)
                {
                    VG_LITE_ERROR_EXIT("Clear failed: vg_lite_clear() returned error %d\r\n", vglite_status);
                }
                __DMB();

                ifx_lcd_display_Rect(NO_CAMERA_IMG_X_POS, NO_CAMERA_IMG_Y_POS, (uint8_t *)no_camera_img_map, NO_CAMERA_IMG_WIDTH, NO_CAMERA_IMG_HEIGHT);              

                Cy_GFXSS_Set_FrameBuffer(base, (uint32_t *)renderTarget->address, &gfx_context);
                __DMB();
            }
        }
        else
        {
            if (Camera_not_supported)
            {
                int i = 0;
                while (i < 2)
                {
                    i++;
                    __DMB();
                    vg_lite_finish();
                    Cy_GFXSS_Set_FrameBuffer(base, (uint32_t *)renderTarget->address, &gfx_context);
                    __DMB();

                    /* Clear the buffer with black color */
                    vglite_status = vg_lite_clear(renderTarget, NULL, BLACK_COLOR);
                    if (VG_LITE_SUCCESS != vglite_status)
                    {
                        VG_LITE_ERROR_EXIT("Clear failed: vg_lite_clear() returned error %d\r\n", vglite_status);
                    }
                    __DMB();

                    ifx_lcd_display_Rect(CAMERA_NOT_SUPPORTED_IMG_X_POS, CAMERA_NOT_SUPPORTED_IMG_Y_POS, (uint8_t *)camera_not_supported_img_map, CAMERA_NOT_SUPPORTED_IMG_WIDTH, CAMERA_NOT_SUPPORTED_IMG_HEIGHT);

                    Cy_GFXSS_Set_FrameBuffer(base, (uint32_t *)renderTarget->address, &gfx_context);
                    __DMB();
                }
                while (_device_connected);
            }
        }

        result = cy_rtos_semaphore_get(&model_semaphore, 0xFFFFFFFF);
        if (CY_RSLT_SUCCESS == result)
        {
            if (_device_connected)
            {
                /* Update bounding box data and inference time */
                update_box_data(renderTarget, &Prediction);
                update_box_data1(renderTarget, Inference_time);

                VG_switch_frame();

            }
        }
    }
}

