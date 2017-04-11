#include "main.h"
#include "configfpga.h"
#include "slavefifo.h"

extern "C" {
#include "ad9364/freesrp.h"
}

#include <cyu3system.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3gpio.h>



CyU3PThread _application_thread;
uint8_t _ep0_buf[32];
uint8_t _ad9364_cmd_response_buf[32];
int _cmd_response_len;

ConfigFpga *_config_fpga_app = NULL;
CyU3PEvent _config_fpga_event;
uint32_t _config_fpga_filelen = 0;

SlaveFifo *_slave_fifo_app = NULL;


void CyFxAppErrorHandler(CyU3PReturnStatus_t apiRetStatus, uint8_t blinkCode)
{
    /* Application failed with the error code apiRetStatus */
    /* Loop Indefinitely */
    int i;

    for(;;)
    {
        for(i = 0; i < blinkCode; i++)
        {
            /* Flash the LED quickly to indicate error */
            CyU3PGpioSetValue(FREESRP_GPIO_LED, 1);
            CyU3PThreadSleep(100);
            CyU3PGpioSetValue(FREESRP_GPIO_LED, 0);
            CyU3PThreadSleep(200);
        }
        CyU3PThreadSleep(500);
    }
}

CyBool_t FreeSRPHandleVendorRequest(uint8_t bRequest, uint8_t bReqType, uint16_t wValue, uint16_t wIndex, uint16_t wLength)
{
    CyBool_t isHandled = CyTrue;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    switch(bRequest)
    {
    case FREESRP_VENDOR_GET_VERSION:
        apiRetStatus = CyU3PUsbSendEP0Data((uint16_t) strlen(FREESRP_VERSION), (uint8_t *) FREESRP_VERSION);
        break;
    case FREESRP_VENDOR_FPGA_CONFIG_LOAD:
        if((bReqType & 0x80) == 0)
        {
            if(!_slave_fifo_app->active) // Don't do anything if FPGA is already configured and this has switched to Slave FIFO mode
            {
                CyU3PUsbGetEP0Data(wLength, _ep0_buf, NULL);
                _config_fpga_filelen = (uint32_t)(_ep0_buf[3]<<24)|(_ep0_buf[2]<<16)|(_ep0_buf[1]<<8)|_ep0_buf[0];

                // Set FREESRP_EVENT_FPGA_CONFIG_START to start configuring FPGA
                CyU3PEventSet(&_config_fpga_event, FREESRP_EVENT_FPGA_CONFIG_START, CYU3P_EVENT_OR);
            }
            isHandled = CyTrue;
        }
        break;
    case FREESRP_VENDOR_FPGA_CONFIG_STATUS:
        if((bReqType & 0x80) == 0x80)
        {
            _ep0_buf[0] = (uint8_t) _config_fpga_app->config_done();
            CyU3PUsbSendEP0Data(wLength, _ep0_buf);
            isHandled = CyTrue;
        }
        break;
    case FREESRP_VENDOR_FPGA_CONFIG_FINISH:
        if((bReqType & 0x80) == 0x80)
        {
            // Switch to slave FIFO interface when FPGA is configured successfully
            if(_config_fpga_app->config_done())
            {
                CyU3PEventSet(&_config_fpga_event, FREESRP_EVENT_FPGA_CONFIG_FINISH, CYU3P_EVENT_OR);
                _ep0_buf[0] = 1;
                CyU3PUsbSendEP0Data(wLength, _ep0_buf);
            }
            else
            {
                _ep0_buf[0] = 0;
                CyU3PUsbSendEP0Data(wLength, _ep0_buf);
            }
            isHandled = CyTrue;
        }
        break;
    case FREESRP_VENDOR_XCVR_CMD:
        if((bReqType & 0x80) == 0)
        {
            CyU3PUsbGetEP0Data(wLength, _ep0_buf, NULL);
            _cmd_response_len = ad9364_cmd(_ep0_buf, _ad9364_cmd_response_buf);
            CyU3PUsbSendEP0Data((uint16_t) _cmd_response_len, _ad9364_cmd_response_buf);

            isHandled = CyTrue;
        }
        break;
    case FREESRP_VENDOR_XCVR_INIT:
        if((bReqType & 0x80) == 0)
        {
            int ad9364_init_status = ad9364_init();
            CyU3PUsbSendEP0Data(sizeof(ad9364_init_status), (uint8_t *) &ad9364_init_status);

            isHandled = CyTrue;
        }
        break;

    default:
        isHandled = CyFalse;
    }

    return isHandled;
}

void CyFxApplicationUSBEventCB(CyU3PUsbEventType_t evtype, uint16_t evdata)
{
    switch (evtype)
    {
    case CY_U3P_USB_EVENT_SETCONF:
        if(_slave_fifo_app->active)
        {
            // Restart application
            _slave_fifo_app->stop();
            CyU3PUsbLPMDisable();
            _slave_fifo_app->start();
        }
        else
        {
            CyU3PUsbLPMDisable();

            // Start FPGA configuration
            _config_fpga_app->start();
        }

        break;

    case CY_U3P_USB_EVENT_RESET:
    case CY_U3P_USB_EVENT_DISCONNECT:
        // Stop the application
        if(_slave_fifo_app->active)
        {
            _slave_fifo_app->stop();
        }
        break;

    default:
        break;
    }
}

CyBool_t CyFxApplicationLPMRequestCB(CyU3PUsbLinkPowerMode link_mode)
{
    // Allow transition to low power mode
    return CyTrue;
}

CyBool_t CyFxApplicationUSBSetupCB(uint32_t setupdat0, uint32_t setupdat1)
{
    /* Fast enumeration is used. Only requests addressed to the interface, class,
     * vendor and unknown control requests are received by this function.
     * This application does not support any class or vendor requests. */
    uint8_t  bRequest, bReqType;
    uint8_t  bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    CyBool_t isHandled = CyFalse;

    /* Decode the fields from the setup request. */
    bReqType = (setupdat0 & CY_U3P_USB_REQUEST_TYPE_MASK);
    bType    = (bReqType & CY_U3P_USB_TYPE_MASK);
    bTarget  = (bReqType & CY_U3P_USB_TARGET_MASK);
    bRequest = ((setupdat0 & CY_U3P_USB_REQUEST_MASK) >> CY_U3P_USB_REQUEST_POS);
    wValue   = ((setupdat0 & CY_U3P_USB_VALUE_MASK)   >> CY_U3P_USB_VALUE_POS);
    wIndex   = ((setupdat1 & CY_U3P_USB_INDEX_MASK)   >> CY_U3P_USB_INDEX_POS);
    wLength  = ((setupdat1 & CY_U3P_USB_LENGTH_MASK)  >> CY_U3P_USB_LENGTH_POS);

    if(bType == CY_U3P_USB_STANDARD_RQT)
    {
        /* Handle SET_FEATURE(FUNCTION_SUSPEND) and CLEAR_FEATURE(FUNCTION_SUSPEND)
         * requests here. It should be allowed to pass if the device is in configured
         * state and failed otherwise. */
        if((bTarget == CY_U3P_USB_TARGET_INTF) && ((bRequest == CY_U3P_USB_SC_SET_FEATURE)
                || (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)) && (wValue == 0))
        {
            if(_slave_fifo_app->active)
            {
                CyU3PUsbAckSetup();
            }
            else
            {
                CyU3PUsbStall(0, CyTrue, CyFalse);
            }

            isHandled = CyTrue;
        }

        /* CLEAR_FEATURE request for endpoint is always passed to the setup callback
         * regardless of the enumeration model used. When a clear feature is received,
         * the previous transfer has to be flushed and cleaned up. This is done at the
         * protocol level. Since this is just a debug log operation, there is no higher
         * level protocol and there are two DMA channels associated with the function,
         * it is easier to stop and restart the application. If there are more than one
         * EP associated with the channel reset both the EPs. The endpoint stall and toggle
         * / sequence number is also expected to be reset. Return CyFalse to make the
         * library clear the stall and reset the endpoint toggle. Or invoke the
         * CyU3PUsbStall (ep, CyFalse, CyTrue) and return CyTrue. Here we are clearing
         * the stall. */
        if((bTarget == CY_U3P_USB_TARGET_ENDPT) && (bRequest == CY_U3P_USB_SC_CLEAR_FEATURE)
                && (wValue == CY_U3P_USBX_FS_EP_HALT))
        {
            if(_slave_fifo_app->active)
            {
                if(((wIndex == FREESRP_USB_ENDPOINT_CMD_IN) || (wIndex == FREESRP_USB_ENDPOINT_CMD_OUT)))
                {
                    _slave_fifo_app->stop();
                    _slave_fifo_app->start();
                    CyU3PUsbStall(wIndex, CyFalse, CyTrue);

                    CyU3PUsbAckSetup();
                    isHandled = CyTrue;
                }

                if(wIndex == FREESRP_USB_ENDPOINT_RX_DATA_IN)
                {
                    CyU3PDmaChannelReset(&_slave_fifo_app->_dma_handle_gpif_to_usb);
                    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_RX_DATA_IN);
                    CyU3PUsbResetEp(FREESRP_USB_ENDPOINT_RX_DATA_IN);
                    CyU3PDmaChannelSetXfer(&_slave_fifo_app->_dma_handle_gpif_to_usb, 0);

                    CyU3PUsbStall(wIndex, CyFalse, CyTrue);
                    isHandled = CyTrue;
                }

                if(wIndex == FREESRP_USB_ENDPOINT_TX_DATA_OUT)
                {
                    CyU3PDmaChannelReset(&_slave_fifo_app->_dma_handle_usb_to_gpif);
                    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_TX_DATA_OUT);
                    CyU3PUsbResetEp(FREESRP_USB_ENDPOINT_TX_DATA_OUT);
                    CyU3PDmaChannelSetXfer(&_slave_fifo_app->_dma_handle_usb_to_gpif, 0);

                    CyU3PUsbStall(wIndex, CyFalse, CyTrue);
                    isHandled = CyTrue;
                }
            }
        }
    }

    if(bType == CY_U3P_USB_VENDOR_RQT)
    {
        isHandled = FreeSRPHandleVendorRequest(bRequest, bReqType, wValue, wIndex, wLength);
    }

    return isHandled;
}

void CyFxApplicationEnumerate()
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Start the USB functionality. */
    apiRetStatus = CyU3PUsbStart();
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* The fast enumeration is the easiest way to setup a USB connection,
     * where all enumeration phase is handled by the library. Only the
     * class / vendor requests need to be handled by the application. */
    CyU3PUsbRegisterSetupCallback(CyFxApplicationUSBSetupCB, CyTrue);

    /* Setup the callback to handle the USB events. */
    CyU3PUsbRegisterEventCallback(CyFxApplicationUSBEventCB);

    /* Register a callback to handle LPM requests from the USB 3.0 host. */
    CyU3PUsbRegisterLPMRequestCallback(CyFxApplicationLPMRequestCB);

    /* Set the USB Enumeration descriptors */

    /* Super speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* High speed device descriptor. */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_DEVICE_DESCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* BOS descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_BOS_DESCR, 0, (uint8_t *)CyFxUSBBOSDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* Device qualifier descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_DEVQUAL_DESCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* Super speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_SS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* High speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_HS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* Full speed configuration descriptor */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_FS_CONFIG_DESCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* String descriptor 0 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* String descriptor 1 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* String descriptor 2 */
    apiRetStatus = CyU3PUsbSetDesc(CY_U3P_USB_SET_STRING_DESCR, 2, (uint8_t *)CyFxUSBProductDscr);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    /* Connect the USB Pins with super speed operation enabled. */
    apiRetStatus = CyU3PConnectState(CyTrue, CyTrue);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }
}

void ApplnThread_Entry(uint32_t input)
{
    uint32_t eventFlag;
    CyU3PReturnStatus_t txApiRetStatus = CY_U3P_SUCCESS;

    // Setup event for starting/finishing FPGA configuration
    txApiRetStatus = CyU3PEventCreate(&_config_fpga_event);
    if(txApiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "event create failed, Error Code = %d\n", txApiRetStatus);
    }

    // Applications
    _config_fpga_app = new ConfigFpga;
    _slave_fifo_app = new SlaveFifo;

    // Initialize the FPGA configuration application
    _config_fpga_app->init();

    // Enumerate
    CyFxApplicationEnumerate();

    for(;;)
    {
        if(_slave_fifo_app->active || _config_fpga_app->active)
        {
            // Wait for events to configure FPGA
            txApiRetStatus = CyU3PEventGet(&_config_fpga_event, (FREESRP_EVENT_FPGA_CONFIG_START | FREESRP_EVENT_FPGA_CONFIG_FINISH), CYU3P_EVENT_OR_CLEAR, &eventFlag, CYU3P_WAIT_FOREVER);

            if(txApiRetStatus == CY_U3P_SUCCESS)
            {
                if(eventFlag & FREESRP_EVENT_FPGA_CONFIG_START)
                {
                    // Event received that signals configuration start request
                    if(!_slave_fifo_app->active)
                    {
                        _config_fpga_app->config_fpga(_config_fpga_filelen);
                    }
                }
                else if((eventFlag & FREESRP_EVENT_FPGA_CONFIG_FINISH))
                {
                    // Event received that signals end of FPGA config and switch to main app request

                    // Stop FPGA config app
                    _config_fpga_app->stop();

                    // Start main app
                    _slave_fifo_app->init();
                    _slave_fifo_app->start();
                }
            }
        }
    }
}

void CyFxApplicationDefine()
{
    void *ptr = NULL;
    uint32_t retThrdCreate = CY_U3P_SUCCESS;

    // Allocate the memory for the threads
    ptr = CyU3PMemAlloc(CY_FX_APPLN_THREAD_STACK);

    // Create the thread for the application
    retThrdCreate = CyU3PThreadCreate(
            &_application_thread,                    /* App Thread structure */
            "21:FreeSRP",                            /* Thread ID and Thread name */
            ApplnThread_Entry,                       /* App Thread Entry function */
            0,                                       /* No input parameter to thread */
            ptr,                                     /* Pointer to the allocated thread stack */
            CY_FX_APPLN_THREAD_STACK,                /* App Thread stack size */
            CY_FX_APPLN_THREAD_PRIORITY,             /* App Thread priority */
            CY_FX_APPLN_THREAD_PRIORITY,             /* App Thread pre-emption threshold */
            CYU3P_NO_TIME_SLICE,                     /* No time slice for the application thread */
            CYU3P_AUTO_START                         /* Start the Thread immediately */
    );

    if(retThrdCreate != 0)
    {
        // Thread Creation failed with the error code retThrdCreate
        // Application cannot continue
        // Loop indefinitely
        while(1);
    }
}

int main()
{
    CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
    CyU3PSysClockConfig_t clkCfg;

    // setSysClk400 clock configuration
    clkCfg.setSysClk400 = CyTrue;   /* FX3 device's master clock is set to a frequency > 400 MHz */
    clkCfg.cpuClkDiv = 2;           /* CPU clock divider */
    clkCfg.dmaClkDiv = 2;           /* DMA clock divider */
    clkCfg.mmioClkDiv = 2;          /* MMIO clock divider */
    clkCfg.useStandbyClk = CyFalse; /* device has no 32KHz clock supplied */
    clkCfg.clkSrc = CY_U3P_SYS_CLK; /* Clock source for a peripheral block  */

    // Initialize the device
    status = CyU3PDeviceInit(&clkCfg);
    if(status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    // Initialize the caches. Enable both Instruction and Data Caches.
    status = CyU3PDeviceCacheControl(CyTrue, CyTrue, CyTrue);
    if(status != CY_U3P_SUCCESS)
    {
        goto handle_fatal_error;
    }

    // This is a non returnable call for initializing the RTOS kernel
    CyU3PKernelEntry();

    return 0;

handle_fatal_error:
    // Cannot recover from this error.
    while(1);
}