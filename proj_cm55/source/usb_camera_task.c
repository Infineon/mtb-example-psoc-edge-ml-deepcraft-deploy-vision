/******************************************************************************
* File Name:   usb_camera_task.c
*
* Description: This file implements the USB Webcam functions.
*
* Related Document: See README.md
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

/*****************************************************************************
 * Header Files
 *****************************************************************************/
#include <inttypes.h>
#include "cybsp.h"
#include "cy_pdl.h"
#include "cyabs_rtos.h"
/* FreeRTOS header file */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "cyabs_rtos.h"
#include "cyabs_rtos_impl.h"
#include "USBH.h"
#include "USBH_Util.h"
#include "USBH_VIDEO.h"
#include <stdint.h>
#include <stdio.h>
#include "definitions.h"
#include "lcd_task.h"
#include "usb_camera_task.h"
#include "ifx_time_utils.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

/*******************************************************************************
 * Global Variables
 *******************************************************************************/
static cy_thread_t usbh_main_task_handle;           // Handle for USB host main task
static cy_thread_t usbh_isr_task_handle;            // Handle for USB host ISR task

static cy_queue_t _VIDEO_MailBox;                   // Mailbox for video-related messages
static cy_queue_t _DeviceStateMailBox;              // Mailbox for device state messages
cy_queue_t _NewFrameMailBox;                        // Mailbox for new frame notifications

VideoBuffer_t _ImageBuff[NUM_IMAGE_BUFFERS];         // Array of image buffers for video frames

uint8_t _device_connected;                          // Flag indicating device connection status
static char _ac[128];                               // Buffer for general-purpose character data
uint8_t lastBuffer;                                 // Index of the last used buffer
bool Logitech_camera_enabled = 0;                    // Flag for Logitech camera enable status
bool point3mp_camera_enabled = 0;                    // Flag for 3MP camera enable status
bool twomp_camera_enabled = 0;                      // Flag for 2MP camera enable status
bool Camera_not_supported = 0;                      // Flag for unsupported camera status
U8 disconnectEvent =0;                               /// Device disconnect event status

// Frame monitoring variables
float last_frame_time = 0;                          // Timestamp of the last frame received
float last_check_time = 0;                          // Timestamp of the last frame check
static int _StreamErrCnt;                           // Counter for stream errors
float frame_timeout_ms = 10000;                     // Timeout for stalled frames (10 seconds)

static USBH_NOTIFICATION_HOOK Hook;                  // USB host notification hook

extern cy_semaphore_t usb_semaphore;                // External semaphore for USB operations
extern vg_lite_buffer_t display_buffer[2];           // External display buffer array
extern vg_lite_buffer_t usb_yuv_frames[NUM_IMAGE_BUFFERS]; // External YUV frame buffers



/*******************************************************************************
* Function Name: _cbOnAddRemoveDevice
*******************************************************************************
 * Description:
 *   Callback function invoked when a USB device is added or removed. Executes in
 *   the context of the USBH_Task and handles device connection/disconnection events
 *   by updating buffers, logging events, and signaling tasks. Non-blocking.
 *
 * Input Arguments:
 *   void *pContext - Context pointer (not used)
 *   U8 DevIndex - Index of the device
 *   USBH_DEVICE_EVENT Event - Event type (add or remove)
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _cbOnAddRemoveDevice(void *pContext, U8 DevIndex, USBH_DEVICE_EVENT Event)
{
    (void)pContext;

    switch (Event)
    {
        case USBH_DEVICE_EVENT_ADD:
            _device_connected = 1;
            USBH_Logf_Application("**** VIDEO Device added [%d]\n", DevIndex);
            for (int i = 0; i < NUM_IMAGE_BUFFERS; i++)
            {
                _ImageBuff[i].BufReady = 0;
                _ImageBuff[i].NumBytes = 0;
            }
            lastBuffer = 0;
            __DMB();
            (void)cy_rtos_queue_put(&_VIDEO_MailBox, &DevIndex, 0);
            break;
        case USBH_DEVICE_EVENT_REMOVE:
            _device_connected = 0;
            USBH_Logf_Application("**** VIDEO Device removed [%d]\n", DevIndex);

            // Reset buffer states
            for (int i = 0; i < NUM_IMAGE_BUFFERS; i++)
            {
                _ImageBuff[i].BufReady = 0;
                _ImageBuff[i].NumBytes = 0;
            }
            __DMB();
            // Signal main task about disconnection
            disconnectEvent = 0xff;
            cy_rtos_queue_put(&_DeviceStateMailBox, &disconnectEvent, 0);
            cy_rtos_semaphore_set(&model_semaphore);
            _StreamErrCnt = 0;
            break;
        default:
            ; // Should never happen
    }
}

CY_SECTION_ITCM_BEGIN
/*******************************************************************************
* Function Name: _cbOnData
*******************************************************************************
 * Description:
 *   Callback function invoked when video data is received from a USB video device.
 *   Manages frame buffering, error handling, and stream state for video data
 *   processing. Updates buffer states, signals semaphores, and handles frame
 *   completion or errors.
 *
 * Input Arguments:
 *   USBH_VIDEO_DEVICE_HANDLE hDevice - Handle to the video device (not used)
 *   USBH_VIDEO_STREAM_HANDLE hStream - Handle to the video stream
 *   USBH_STATUS Status - Status of the data transfer
 *   const U8 *pData - Pointer to the received data
 *   unsigned NumBytes - Number of bytes received
 *   U32 Flags - Flags indicating frame status
 *   void *pUserDataContext - User context pointer (not used)
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _cbOnData(USBH_VIDEO_DEVICE_HANDLE hDevice, USBH_VIDEO_STREAM_HANDLE hStream, USBH_STATUS Status, const U8 *pData, unsigned NumBytes, U32 Flags, void *pUserDataContext)
{
    (void)pData;
    (void)hDevice;
    (void)pUserDataContext;

    static size_t _frame_bytes = 0;
    static uint8_t _throw_away_frame = 0;
    static uint8_t _current_Buffer = 0;
    static uint16_t _frame_count = 0;
    static uint32_t _error_count = 0;
    USBH_STATUS Status1;
    I8 IsStreamStopped;

    /* Check if USB is ready to transmit */
    if (Status == USBH_STATUS_SUCCESS)
    {
        _StreamErrCnt = 0;
        _error_count = 0;

        bool endOfFrame = (Flags & USBH_UVC_END_OF_FRAME) == USBH_UVC_END_OF_FRAME;

        /* Process every N-th frame only to reduce computational load ; 
        (_frame_count / N) FPS */
        if (!point3mp_camera_enabled)
        {
            if (!Logitech_camera_enabled)
                _throw_away_frame |= (_frame_count % FRAMES_TO_SKIP);
            else
                _throw_away_frame |= (_frame_count % FRAMES_TO_SKIP_LOGITECH);
        }

        if (_frame_bytes + NumBytes > CAMERA_BUFFER_SIZE)
        {
            printf("[USB Camera] Buffer overflow: %u (%6u)\r\n", _current_Buffer, _frame_bytes + NumBytes);
            _throw_away_frame = 1;
            _frame_bytes = 0;
        }

        if (_throw_away_frame)
        {
            if (endOfFrame)
            {
                _frame_count++;
                _frame_bytes = 0;
                _throw_away_frame = _ImageBuff[_current_Buffer].BufReady;
            }
            USBH_VIDEO_Ack(hStream);
            return;
        }

        // Copy data into the current buffer
        memcpy((uint8_t *)usb_yuv_frames[_current_Buffer].memory + _frame_bytes, pData, NumBytes);
        _frame_bytes += NumBytes;

        // Switch buffers when a buffer is full or when we register an end-of-frame
        if (endOfFrame)
        {
            _frame_count++;

            if (_frame_bytes == CAMERA_BUFFER_SIZE)
            {
                /* Processing of the previous frame is DONE, no ready buffer, and wait for a new ready frame */
                lastBuffer = _current_Buffer;
                _ImageBuff[_current_Buffer].BufReady = 1;
                _ImageBuff[_current_Buffer].NumBytes = _frame_bytes;

                __DMB();

                // Switch to next buffer
                _current_Buffer = (_current_Buffer + 1) % NUM_IMAGE_BUFFERS;
                /* If there is only ONE buffer, throw away next frames until processing is completed */
                _throw_away_frame = _ImageBuff[_current_Buffer].BufReady;
                if (!Logitech_camera_enabled)
                {
                    cy_rslt_t result = cy_rtos_semaphore_get(&usb_semaphore, 0xFFFFFFFF);
                    if (CY_RSLT_SUCCESS != result)
                    {
                        printf("[USB Camera] USB Semaphore get failed\r\n");
                    }
                }
                else
                {
                    cy_rtos_semaphore_set(&usb_semaphore);
                }
            }
            // else /* short frame */

            _frame_bytes = 0;

        }

        USBH_VIDEO_Ack(hStream);

    }
    else
    {
        _error_count++;
        if (_error_count % 100 == 1)
        {
            printf("[USB] Transfer error %s (count: %" PRIu32 ")\n", USBH_GetStatusStr(Status), _error_count);
        }

        _StreamErrCnt++;
        if (_StreamErrCnt >= 10)
        {
            Status1 = USBH_STATUS_DEVICE_ERROR; // Indicate failure
        }
        else
        {
            if (Status != USBH_STATUS_DEVICE_REMOVED)
            {
                Status1 = USBH_VIDEO_GetStreamState(hStream, &IsStreamStopped);
                if (Status1 == USBH_STATUS_SUCCESS)
                {
                    if (IsStreamStopped == 1)
                    {
                        Status1 = USBH_VIDEO_RestartStream(hStream);
                        if (Status1 == USBH_STATUS_SUCCESS)
                        {
                            USBH_Logf_Application("_cbOnData: Restarted video stream after packet error");
                        }
                        else
                        {
                            USBH_Logf_Application("_cbOnData: USBH_VIDEO_RestartStream failed %s", USBH_GetStatusStr(Status));
                        }
                    }
                    else
                    {
                        USBH_Warnf_Application("_cbOnData: _StreamErrCnt %d, but stream is running", _StreamErrCnt);
                    }
                }
                else
                {
                    USBH_Logf_Application("_cbOnData: USBH_VIDEO_GetStreamState failed %s", USBH_GetStatusStr(Status));
                }
            }
        }

        // If the consecutive error count reaches max, the device was disconnected, or the stream could not be restarted - remove the video device
        if ((Status == USBH_STATUS_DEVICE_REMOVED) || (Status1 != USBH_STATUS_SUCCESS))
        {
            USBH_Logf_Application("_cbOnData: device removed or max conseq. errors exceeded");
            U8 MBEvent = 0xff; // Device disconnected or had an error
            cy_rtos_queue_put(&_DeviceStateMailBox, &MBEvent, 0);
            _StreamErrCnt = 0;
        }

        // Reset frame state on error but don't change buffer states
        _frame_bytes = 0;
        _throw_away_frame = 0;
        USBH_VIDEO_Ack(hStream);
        return;
    }
}
CY_SECTION_ITCM_END

/*******************************************************************************
* Function Name: _GetFrameIntervals
*******************************************************************************
 * Description:
 *   Retrieves and formats frame interval information for a video frame into a
 *   string buffer for logging or display purposes.
 *
 * Input Arguments:
 *   USBH_VIDEO_FRAME_INFO *pFrame - Pointer to the frame information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _GetFrameIntervals(USBH_VIDEO_FRAME_INFO *pFrame)
{
    char ac[32];
    unsigned i;

    USBH_MEMSET(_ac, 0, sizeof(_ac));
    USBH_MEMSET(ac, 0, sizeof(ac));
    for (i = 0; i < pFrame->bFrameIntervalType; i++)
    {
        if (i == 0)
        {
            SEGGER_snprintf(ac, sizeof(ac), "%lu", pFrame->u.dwFrameInterval[i]);
        }
        else
        {
            SEGGER_snprintf(ac, sizeof(ac), ", %lu", pFrame->u.dwFrameInterval[i]);
        }
        strncat(_ac, ac, sizeof(_ac) - (strlen(_ac) + 1));
    }
}

/*******************************************************************************
 * Function Name: _TermType2Str
 *******************************************************************************
 * Description:
 *   Converts a terminal type code to a human-readable string representation.
 *
 * Input Arguments:
 *   U8 Type - Terminal type code
 *
 * Return Value:
 *   const char* - String representation of the terminal type
 *
 *******************************************************************************/
static const char* _TermType2Str(U8 Type)
{
    switch (Type)
    {
        case USBH_VIDEO_VC_INPUT_TERMINAL:
            return "Input";
        case USBH_VIDEO_VC_OUTPUT_TERMINAL:
            return "Output";
        case USBH_VIDEO_VC_SELECTOR_UNIT:
            return "Selector";
        case USBH_VIDEO_VC_PROCESSING_UNIT:
            return "Processing";
        case USBH_VIDEO_VC_EXTENSION_UNIT:
            return "Extension";
        default:
            return "Unknown";
    }
}

/*******************************************************************************
 * Function Name: _FormatType2Str
 *******************************************************************************
 * Description:
 *   Converts a video format type code to a human-readable string representation.
 *
 * Input Arguments:
 *   U8 Type - Video format type code
 *
 * Return Value:
 *   const char* - String representation of the video format type
 *
 *******************************************************************************/
static const char* _FormatType2Str(U8 Type)
{
    switch (Type)
    {
        case USBH_VIDEO_VS_FORMAT_UNCOMPRESSED:
            return "Uncompressed";
        case USBH_VIDEO_VS_FORMAT_MJPEG:
            return "MJPEG";
        case USBH_VIDEO_VS_FORMAT_FRAME_BASED:
            return "H.264";
        default:
            return "Unknown";
    }
}

/*******************************************************************************
 * Function Name: _FrameType2Str
 *******************************************************************************
 * Description:
 *   Converts a video frame type code to a human-readable string representation.
 *
 * Input Arguments:
 *   U8 Type - Video frame type code
 *
 * Return Value:
 *   const char* - String representation of the video frame type
 *
 *******************************************************************************/
static const char* _FrameType2Str(U8 Type)
{
    switch (Type)
    {
        case USBH_VIDEO_VS_FRAME_UNCOMPRESSED:
            return "Uncompressed";
        case USBH_VIDEO_VS_FRAME_MJPEG:
            return "MJPEG";
        case USBH_VIDEO_VS_FRAME_FRAME_BASED:
            return "H.264";
        default:
            return "Unknown";
    }
}

/*******************************************************************************
 * Function Name: _PrintInputTermInfo
 *******************************************************************************
 * Description:
 *   Logs information about an input terminal, specifically for camera terminals.
 *
 * Input Arguments:
 *   USBH_VIDEO_TERM_UNIT_INFO *pTermInfo - Pointer to the terminal information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _PrintInputTermInfo(USBH_VIDEO_TERM_UNIT_INFO *pTermInfo)
{
    if (pTermInfo->u.Terminal.TerminalType != USBH_VIDEO_ITT_CAMERA)
    {
        USBH_Logf_Application("  Input terminal type 0x%x is not handled...", pTermInfo->u.Terminal.TerminalType);
    }
    else
    {
        USBH_Logf_Application("  Associated terminal ID %d", pTermInfo->u.Terminal.u.CameraTerm.bAssocTerminal);
        USBH_Logf_Application(
                "  Objective focal length min %d",
                pTermInfo->u.Terminal.u.CameraTerm.wObjectiveFocalLengthMin);
        USBH_Logf_Application(
                "  Objective focal length max %d",
                pTermInfo->u.Terminal.u.CameraTerm.wObjectiveFocalLengthMax);
        USBH_Logf_Application("  Ocular focal length %d", pTermInfo->u.Terminal.u.CameraTerm.wOcularFocalLength);
        USBH_Logf_Application("  Control Size %d", pTermInfo->u.Terminal.u.CameraTerm.bControlSize);
    }
}

/*******************************************************************************
 * Function Name: _PrintOutputTermInfo
 *******************************************************************************
 * Description:
 *   Logs information about an output terminal, specifically for streaming terminals.
 *
 * Input Arguments:
 *   USBH_VIDEO_TERM_UNIT_INFO *pTermInfo - Pointer to the terminal information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _PrintOutputTermInfo(USBH_VIDEO_TERM_UNIT_INFO *pTermInfo)
{
    if (pTermInfo->u.Terminal.TerminalType != USBH_VIDEO_TT_STREAMING)
    {
        USBH_Logf_Application("  Ouput terminal type 0x%x is not handled...", pTermInfo->u.Terminal.TerminalType);
    }
    else
    {
        USBH_Logf_Application("  Associated terminal ID %d", pTermInfo->u.Terminal.u.OutputTerm.bAssocTerminal);
        USBH_Logf_Application("  Source ID %d", pTermInfo->u.Terminal.u.OutputTerm.bSourceID);
    }
}

/*******************************************************************************
 * Function Name: _PrintSelectorUnitInfo
 *******************************************************************************
 * Description:
 *   Logs information about a selector unit, including its input pins.
 *
 * Input Arguments:
 *   USBH_VIDEO_TERM_UNIT_INFO *pTermInfo - Pointer to the terminal information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _PrintSelectorUnitInfo(USBH_VIDEO_TERM_UNIT_INFO *pTermInfo)
{
    unsigned i;

    USBH_Logf_Application("  Unit ID %d", pTermInfo->bTermUnitID);
    USBH_Logf_Application("  Number of pins %d", pTermInfo->u.SelectUnit.bNrInPins);
    for (i = 0; i < pTermInfo->u.SelectUnit.bNrInPins; i++)
    {
        USBH_Logf_Application("  Input pin: %d", pTermInfo->u.SelectUnit.baSourceID[i]);
    }
}

/*******************************************************************************
 * Function Name: _PrintProcessingUnitInfo
 *******************************************************************************
 * Description:
 *   Logs information about a processing unit, including its source and control details.
 *
 * Input Arguments:
 *   USBH_VIDEO_TERM_UNIT_INFO *pTermInfo - Pointer to the terminal information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _PrintProcessingUnitInfo(USBH_VIDEO_TERM_UNIT_INFO *pTermInfo)
{
    USBH_Logf_Application("  Unit ID %d", pTermInfo->bTermUnitID);
    USBH_Logf_Application("  Source ID %d", pTermInfo->u.ProcessUnit.bSourceID);
    USBH_Logf_Application("  Max multiplier %d", pTermInfo->u.ProcessUnit.wMaxMultiplier);
    USBH_Logf_Application("  Control size %d", pTermInfo->u.ProcessUnit.bControlSize);
}

/*******************************************************************************
 * Function Name: _PrintExtensionUnitInfo
 *
 * Description:
 *   Logs information about an extension unit, including its GUID, pins, and controls.
 *
 * Input Arguments:
 *   USBH_VIDEO_TERM_UNIT_INFO *pTermInfo - Pointer to the terminal information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _PrintExtensionUnitInfo(USBH_VIDEO_TERM_UNIT_INFO *pTermInfo)
{
    USBH_Logf_Application("  Unit ID %d", pTermInfo->bTermUnitID);
    USBH_Logf_Application("  Unit GUID 0x%x...", pTermInfo->u.ExtUnit.guidExtensionCode[0]);
    USBH_Logf_Application("  Number of pins %d", pTermInfo->u.ExtUnit.bNrInPins);
    USBH_Logf_Application("  Controls size %d", pTermInfo->u.ExtUnit.bControlSize);
}

/*******************************************************************************
 * Function Name: _PrintTermUnitInfo
 *******************************************************************************
 * Description:
 *   Logs information about a terminal or unit based on its type, delegating to
 *   specific print functions for different unit types. Adds a small delay to
 *   prevent flooding terminal output.
 *
 * Input Arguments:
 *   USBH_VIDEO_TERM_UNIT_INFO *pTermInfo - Pointer to the terminal or unit information structure
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _PrintTermUnitInfo(USBH_VIDEO_TERM_UNIT_INFO *pTermInfo)
{
    USBH_Logf_Application("  Terminal/Unit ID %d type %s", pTermInfo->bTermUnitID, _TermType2Str(pTermInfo->Type));
    switch (pTermInfo->Type)
    {
        case USBH_VIDEO_VC_INPUT_TERMINAL:
            _PrintInputTermInfo(pTermInfo);
            break;
        case USBH_VIDEO_VC_OUTPUT_TERMINAL:
            _PrintOutputTermInfo(pTermInfo);
            break;
        case USBH_VIDEO_VC_SELECTOR_UNIT:
            _PrintSelectorUnitInfo(pTermInfo);
            break;
        case USBH_VIDEO_VC_PROCESSING_UNIT:
            _PrintProcessingUnitInfo(pTermInfo);
            break;
        case USBH_VIDEO_VC_EXTENSION_UNIT:
            _PrintExtensionUnitInfo(pTermInfo);
            break;
        default:
            USBH_Logf_Application("  Type %d is not handled...", pTermInfo->Type);
            break;
    }
    USBH_Logf_Application("---------------");
    cy_rtos_delay_milliseconds(10); // Small delay so that we do not flood terminal output
}


/*******************************************************************************
 * Function Name: _OnDevReady
*******************************************************************************
 * Description:
 *   Handles the initialization and configuration of a USB video device when it is
 *   ready. Enumerates terminals, formats, and frames, configures the video stream,
 *   and manages frame reception until the device is disconnected or an error occurs.
 *
 * Input Arguments:
 *   U8 DevIndex - Index of the video device
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
static void _OnDevReady(U8 DevIndex)
{
    USBH_VIDEO_INPUT_HEADER_INFO InputHeaderInfo;
    USBH_VIDEO_DEVICE_HANDLE hDevice;
    USBH_VIDEO_INTERFACE_INFO IfaceInfo;
    USBH_VIDEO_TERM_UNIT_INFO TermInfo;
    USBH_VIDEO_STREAM_CONFIG StreamInfo;
    USBH_VIDEO_COLOR_INFO ColorInfo;
    USBH_VIDEO_STREAM_HANDLE Stream;
    USBH_VIDEO_FORMAT_INFO Format;
    USBH_VIDEO_FRAME_INFO Frame;
    USBH_STATUS Status;
    unsigned RequestedFrameIntervalIdx;
    unsigned RequestedFormatIdx;
    unsigned RequestedFrameIdx;
    unsigned NumFrameDescriptors;
    unsigned Found;
    unsigned i;
    unsigned j;
    unsigned k;
    int r;
    U8 MBEvent;
    U32 FrameIntervalfrmvidpid;

    // Open the device, the device index is retrieved from the notification callback
    RequestedFormatIdx = 0;
    RequestedFrameIdx = 0;
    RequestedFrameIntervalIdx = 0;
    Found = 0;

    memset(&hDevice, 0, sizeof(hDevice));
    memset(&Stream, 0, sizeof(Stream));
    // Add timeout tracking for device initialization
    float timeout_start_time = ifx_time_get_ms_f();
    float timeout_ms = 5000; // 5 seconds timeout

    Status = USBH_VIDEO_Open(DevIndex, &hDevice);
    if (Status != USBH_STATUS_SUCCESS)
    {
        USBH_Logf_Application("USBH_VIDEO_Open returned with error: %s", USBH_GetStatusStr(Status));
        return;
    }

    Status = USBH_VIDEO_GetInterfaceInfo(hDevice, &IfaceInfo);
    if (Status != USBH_STATUS_SUCCESS)
    {
        USBH_Logf_Application("USBH_VIDEO_GetInterfaceInfo returned with error: %s", USBH_GetStatusStr(Status));
        return;
    }
    USBH_Logf_Application("====================================================================");
    USBH_Logf_Application("Vendor  Id = 0x%0.4X", IfaceInfo.VendorId);
    USBH_Logf_Application("Product Id = 0x%0.4X", IfaceInfo.ProductId);

    if ((LOGI_TECH_C920_VID == IfaceInfo.VendorId) && (LOGI_TECH_C920_PID == IfaceInfo.ProductId))
    {
        Camera_not_supported = 0;
        FrameIntervalfrmvidpid = FRAME_INTERVAL_1;
        Logitech_camera_enabled = 1;
        point3mp_camera_enabled = 0;
        twomp_camera_enabled = 0;
    }
    else if ((HBV_CAM_0P3_VID == IfaceInfo.VendorId) && (HBV_CAM_0P3_PID == IfaceInfo.ProductId))
    {
        Camera_not_supported = 0;
        FrameIntervalfrmvidpid = FRAME_INTERVAL_2;
        Logitech_camera_enabled = 0;
        point3mp_camera_enabled = 1;
        twomp_camera_enabled = 0;
    }
    else if ((HBV_CAM_2P0_VID == IfaceInfo.VendorId) && (HBV_CAM_2P0_PID == IfaceInfo.ProductId))
    {
        Camera_not_supported = 0;
        FrameIntervalfrmvidpid = FRAME_INTERVAL;
        Logitech_camera_enabled = 0;
        point3mp_camera_enabled = 0;
        twomp_camera_enabled = 1;
    }
    else
    {
        cy_rtos_semaphore_set(&model_semaphore);
        Camera_not_supported = 1;
        return;
    }

    // List all terminals/units
    for (i = 0; i < IfaceInfo.NumTermUnits; i++)
    {
        // Check if we've been waiting too long for initialization
        if ((ifx_time_get_ms_f() - timeout_start_time) > timeout_ms)
        {
            USBH_Logf_Application("Timeout waiting for device initialization during terminal enumeration. Resetting...");
            Found = 0;
            goto cleanup;
        }

        Status = USBH_VIDEO_GetTermUnitInfo(hDevice, i, &TermInfo);
        if (Status == USBH_STATUS_SUCCESS)
        {
            _PrintTermUnitInfo(&TermInfo);
        }
        else
        {
            USBH_Logf_Application("USBH_VIDEO_GetTermUnitInfo returned with error: %s", USBH_GetStatusStr(Status));
        }
    }

    // This sets the text information fields and the webcam frame window visible
    memset(&IfaceInfo, 0, sizeof(IfaceInfo));
    if (Status != USBH_STATUS_SUCCESS && Status != USBH_STATUS_NOT_FOUND)
    {
        USBH_Logf_Application("USBH_VIDEO_GetFirstTermUnitInfo returned with error: %s", USBH_GetStatusStr(Status));
    }
    else
    {
        // Configure the VIDEO device
        Status = USBH_VIDEO_GetInputHeader(hDevice, &InputHeaderInfo);
        if (Status != USBH_STATUS_SUCCESS)
        {
            USBH_Logf_Application("USBH_VIDEO_GetInputHeader returned with error: %s", USBH_GetStatusStr(Status));
        }
        else
        {
            USBH_Logf_Application(
                    "Video Interface with index %d (%d formats reported, still capture method %d):",
                    DevIndex,
                    InputHeaderInfo.bNumFormats,
                    InputHeaderInfo.bStillCaptureMethod);
            // Iterate over all formats
            for (i = 0; i < InputHeaderInfo.bNumFormats; i++)
            {
                // Check if we've been waiting too long for initialization
                if ((ifx_time_get_ms_f() - timeout_start_time) > timeout_ms)
                {
                    USBH_Logf_Application("Timeout waiting for device initialization during format discovery. Resetting...");
                    Found = 0;
                    break;
                }

                Status = USBH_VIDEO_GetFormatInfo(hDevice, i, &Format);
                if (Status != USBH_STATUS_SUCCESS)
                {
                    USBH_Logf_Application(
                            "USBH_VIDEO_GetFormatInfo returned with error: %s",
                            USBH_GetStatusStr(Status));
                }
                else
                {
                    switch (Format.FormatType)
                    {
                        case USBH_VIDEO_VS_FORMAT_UNCOMPRESSED:
                            NumFrameDescriptors = Format.u.UncompressedFormat.bNumFrameDescriptors;
                            break;
                        case USBH_VIDEO_VS_FORMAT_MJPEG:
                            NumFrameDescriptors = Format.u.MJPEG_Format.bNumFrameDescriptors;
                            break;
                        case USBH_VIDEO_VS_FORMAT_FRAME_BASED:
                            NumFrameDescriptors = Format.u.H264_Format.bNumFrameDescriptors;
                            break;
                        default:
                            USBH_Logf_Application("  Format type %d is not supported!:", Format.FormatType);
                            Status = USBH_STATUS_ERROR;
                    }

                    // If we have a valid format type - proceed with parsing
                    if (Status == USBH_STATUS_SUCCESS)
                    {
                        USBH_Logf_Application("  Format Index %d, type %s:", i, _FormatType2Str(Format.FormatType));
                        // Check if the format has a color matching descriptor
                        Status = USBH_VIDEO_GetColorMatchingInfo(hDevice, i, &ColorInfo);
                        if (Status != USBH_STATUS_SUCCESS)
                        {
                            USBH_Logf_Application("  No color matching descriptor (%s)", USBH_GetStatusStr(Status));
                        }
                        else
                        {
                            USBH_Logf_Application(
                                    "  Color matching descriptor: bColorPrimaries 0x%x, bTransferCharacteristics 0x%x, bMatrixCoefficients 0x%x",
                                    ColorInfo.bColorPrimaries,
                                    ColorInfo.bTransferCharacteristics,
                                    ColorInfo.bMatrixCoefficients);
                        }

                        // Iterate over all frame types
                        for (j = 0; j < NumFrameDescriptors; j++)
                        {
                            // Check timeout during frame discovery
                            if ((ifx_time_get_ms_f() - timeout_start_time) > timeout_ms)
                            {
                                USBH_Logf_Application("Timeout waiting for device initialization during frame discovery. Resetting...");
                                Found = 0;
                                break;
                            }

                            Status = USBH_VIDEO_GetFrameInfo(hDevice, i, j, &Frame);
                            if (Status != USBH_STATUS_SUCCESS)
                            {
                                USBH_Logf_Application(
                                        "USBH_VIDEO_GetFrameInfo returned with error: %s",
                                        USBH_GetStatusStr(Status));
                            }
                            else
                            {
                                if (Frame.bFrameIntervalType == 0)
                                {
                                    // Frame interval type zero (continuous frame interval) is almost never used by any device
                                    USBH_Logf_Application(
                                            "    Frame Index %d, type %s, x %d, y %d, %d min frame interval, %d max frame interval, %d interval step",
                                            j,
                                            _FrameType2Str(Frame.FrameType),
                                            Frame.wWidth,
                                            Frame.wHeight,
                                            Frame.bFrameIntervalType,
                                            Frame.u.dwMinFrameInterval,
                                            Frame.u.dwMaxFrameInterval,
                                            Frame.u.dwFrameIntervalStep);
                                }
                                else
                                {
                                    _GetFrameIntervals(&Frame);
                                    USBH_Logf_Application(
                                            "    Frame Index %d, type %s, x %d, y %d, intervals (%d): { %s }",
                                            j,
                                            _FrameType2Str(Frame.FrameType),
                                            Frame.wWidth,
                                            Frame.wHeight,
                                            Frame.bFrameIntervalType,
                                            _ac);
                                    // Try to find a setting which matches our defines
                                    if (Format.FormatType == FORMAT)
                                    {
                                        if ((Frame.wHeight == CAMERA_HEIGHT) && (Frame.wWidth == CAMERA_WIDTH))
                                        {
                                            for (k = 0; k < Frame.bFrameIntervalType; k++)
                                            {
                                                if (Frame.u.dwFrameInterval[k] == FrameIntervalfrmvidpid)
                                                {
                                                    if (Found != 1u)
                                                    {
                                                        RequestedFormatIdx = i;
                                                        RequestedFrameIdx = j;
                                                        RequestedFrameIntervalIdx = k + 1;
                                                        Found = 1;
                                                        USBH_Logf_Application(
                                                                "--->Found requested setting (%dx%d @ %d FPS): Format Index %d, Frame Index %d, Frame Interval Index %d",
                                                                CAMERA_WIDTH,
                                                                CAMERA_HEIGHT,
                                                                10000000 / Frame.u.dwFrameInterval[k],
                                                                RequestedFormatIdx,
                                                                RequestedFrameIdx,
                                                                RequestedFrameIntervalIdx);
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                                cy_rtos_delay_milliseconds(10); // Small delay so that we do not flood terminal output
                            }
                        }
                    }
                }
            }

            // If we didn't find an exact format match, try to use first available format
            if (Found != 1u && InputHeaderInfo.bNumFormats > 0)
            {
                USBH_Logf_Application("WARNING: Couldn't find exact requested format. Using first available format instead.");
                // Fall back to first available format if any exist
                RequestedFormatIdx = 0;
                RequestedFrameIdx = 0;
                RequestedFrameIntervalIdx = 1; // First interval
                Found = 1;
            }
        }

        // Receive frame data
        memset(&StreamInfo, 0, sizeof(USBH_VIDEO_STREAM_CONFIG));
        StreamInfo.Flags = 0;
        if (Found == 1u)
        {
            StreamInfo.FormatIdx = RequestedFormatIdx;
            StreamInfo.FrameIdx = RequestedFrameIdx;
            StreamInfo.FrameIntervalIdx = RequestedFrameIntervalIdx;
            StreamInfo.pfDataCallback = _cbOnData;

            // Initialize the frame monitoring variables
            lastBuffer = 0;
            last_check_time = ifx_time_get_ms_f();
            cy_rtos_queue_reset(&_DeviceStateMailBox);
            Status = USBH_VIDEO_OpenStream(hDevice, &StreamInfo, &Stream);
            if (Status != USBH_STATUS_SUCCESS)
            {
                USBH_Logf_Application("USBH_VIDEO_OpenStream returned with error: %s", USBH_GetStatusStr(Status));
            }
            else
            {
                // Wait until the device has been disconnected
                for (;;)
                {
                    // Check for stalled camera frames
                    float current_time = ifx_time_get_ms_f();
                    if ((current_time - last_check_time) > 1000)
                    {
                        last_check_time = current_time;

#ifdef ENABLE_USB_WEBCAM_TIMEOUT
                        if (last_frame_time > 0 && (current_time - last_frame_time) > frame_timeout_ms)
                        {
                            printf("[ERROR] No frames received for %.2f seconds, camera is stalled", frame_timeout_ms / 1000.0);
                            // Force camera reset
                            break;
                        }
#endif // ENABLE_USB_WEBCAM_TIMEOUT
                    }
                    // Checking if the queue operation was successful
                    r = cy_rtos_queue_get(&_DeviceStateMailBox, (char *)&MBEvent, 100);
                    if (r == CY_RSLT_SUCCESS)
                    {
                        if (MBEvent == 0xff)
                        {
                            USBH_Logf_Application("USBH_VIDEO_OpenStream breaking - Device was disconnected");
                            break;
                        }
                        else if (MBEvent == 0xfe)
                        {
                            USBH_Logf_Application("USBH_VIDEO_OpenStream breaking - Transfer status error");
                            break;
                        }
                        else
                        {
                            USBH_Logf_Application("USBH_VIDEO_OpenStream received unknown error code: 0x%02x", MBEvent);
                            if (_device_connected)
                                MBEvent = 0xfe;
                            break;
                        }
                    }
                }
            }
        }
        else
        {
            // Display error text until webcam is removed
            while (_device_connected)
            {
                USBH_OS_Delay(50);
            }
        }
    }

cleanup:
    // Close the device
    USBH_Logf_Application("Closing video device...");
    USBH_VIDEO_Close(hDevice);
}

/*******************************************************************************
 * Function Name: cm55_usb_webcam_task
*******************************************************************************
 * Description:
 *   Main task for handling USB webcam operations. Initializes USB host and video
 *   subsystems, creates tasks for USB handling, sets up mailboxes, and processes
 *   device readiness events from the video mailbox.
 *
 * Input Arguments:
 *   void *arg - Task argument (not used)
 *
 * Return Value:
 *   None
 *
 *******************************************************************************/
void cm55_usb_webcam_task(void *arg)
{
    const uint32_t task_stack_size_bytes = 8192;
    U8 DevIndex;
    cy_rslt_t result;

    USBH_Init();
    result = cy_rtos_create_thread(&usbh_main_task_handle, (void*)USBH_Task, "USBH_Task",
                                   NULL, task_stack_size_bytes, (uint8_t)TASK_PRIO_USBH_MAIN, NULL);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    result = cy_rtos_create_thread(&usbh_isr_task_handle, (void*)USBH_ISRTask, "USBH_isr",
                                   NULL, task_stack_size_bytes, (uint8_t)TASK_PRIO_USBH_ISR, NULL);
    if ( CY_RSLT_SUCCESS != result ) {
        CY_ASSERT(0);
    }

    result = cy_rtos_queue_init(&_VIDEO_MailBox, sizeof(U8), MAX_VIDEO_INTERFACES);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    result = cy_rtos_queue_init(&_DeviceStateMailBox, sizeof(U8), 1);
    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    USBH_VIDEO_Init();
    USBH_VIDEO_AddNotification(&Hook, _cbOnAddRemoveDevice, NULL);

    while (true)
    {
        if (CY_RSLT_SUCCESS == cy_rtos_queue_get(&_VIDEO_MailBox, &DevIndex, 100))
        {
            _OnDevReady(DevIndex);
        }
    }
}

