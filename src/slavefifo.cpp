#include "slavefifo.h"
#include "main.h"
#include "cyfxgpif2config.h"

#include <cyu3system.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3lpp.h>
#include <cyu3gpio.h>
#include <cyu3uart.h>
#include <cyu3pib.h>
#include <cyu3gpif.h>

SlaveFifo::SlaveFifo() : active(false)
{}

void SlaveFifo::init()
{
    init_gpio();
    setup_uart();
    init_gpif();
}

void SlaveFifo::start()
{
    uint16_t fpgaUartSize = 0, rxTxSize = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    // First identify the usb speed. Once that is identified,
    // create a DMA channel and start the transfer on this.

    // Based on the Bus Speed configure the endpoint packet size
    switch(usbSpeed)
    {
    case CY_U3P_FULL_SPEED:
        fpgaUartSize = 64;
        rxTxSize = 64;
        break;

    case CY_U3P_HIGH_SPEED:
        fpgaUartSize = 64;
        rxTxSize = 512;
        break;

    case CY_U3P_SUPER_SPEED:
        fpgaUartSize = 64;
        rxTxSize = 1024;
        break;

    default:
        CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 1);
        break;
    }

    // Set up UART endpoints
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_INTR;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = fpgaUartSize;

    // Configure CMD_IN endpoint
    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_CMD_IN, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Configure CMD_OUT endpoint
    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_CMD_OUT, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Flush the Endpoint memory
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_CMD_IN);
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_CMD_OUT);

    // Set up DMA for CMD_IN
    dmaCfg.size = 16;
    dmaCfg.count = 1;
    dmaCfg.prodSckId = CY_U3P_LPP_SOCKET_UART_PROD;
    dmaCfg.consSckId = FREESRP_USB_SOCKET_CMD_IN;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate(&_dma_handle_uart_to_usb, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Set up DMA for CMD_OUT
    dmaCfg.size = fpgaUartSize;
    dmaCfg.prodSckId = FREESRP_USB_SOCKET_CMD_OUT;
    dmaCfg.consSckId = CY_U3P_LPP_SOCKET_UART_CONS;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;

    apiRetStatus = CyU3PDmaChannelCreate(&_dma_handle_usb_to_uart, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Set DMA Channel transfer size
    apiRetStatus = CyU3PDmaChannelSetXfer(&_dma_handle_uart_to_usb, 0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer(&_dma_handle_usb_to_uart, 0);
    if (apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Set up RX/TX GPIF endpoints
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = rxTxSize;

    // Configure RX_DATA_IN endpoint
    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_RX_DATA_IN, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Configure TX_DATA_OUT endpoint
    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_TX_DATA_OUT, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Flush the Endpoint memory
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_RX_DATA_IN);
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_TX_DATA_OUT);

    // Create a DMA AUTO channel for U2P transfer.
    dmaCfg.size = rxTxSize;
    dmaCfg.count = 32;
    dmaCfg.prodSckId = FREESRP_USB_SOCKET_TX_DATA_OUT;
    dmaCfg.consSckId = FREESRP_GPIF_SOCKET_TX_DATA;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate(&_dma_handle_usb_to_gpif, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 11);
    }

    // Create a DMA AUTO channel for P2U transfer.
    dmaCfg.size = rxTxSize;
    dmaCfg.count = 32;
    dmaCfg.prodSckId = FREESRP_GPIF_SOCKET_RX_DATA;
    dmaCfg.consSckId = FREESRP_USB_SOCKET_RX_DATA_IN;
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;

    apiRetStatus = CyU3PDmaChannelCreate(&_dma_handle_gpif_to_usb, CY_U3P_DMA_TYPE_AUTO, &dmaCfg);

    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 11);
    }

    // Set DMA channel transfer size
    apiRetStatus = CyU3PDmaChannelSetXfer(&_dma_handle_gpif_to_usb, 0);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    apiRetStatus = CyU3PDmaChannelSetXfer(&_dma_handle_usb_to_gpif, 0);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Update the status flag.
    active = CyTrue;
}

void SlaveFifo::init_gpio()
{
    CyU3PGpioClock_t gpioClock;
    CyU3PIoMatrixConfig_t ioCfg;
    CyU3PGpioSimpleConfig_t gpioConfigLed, gpioConfigDone;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    // Init the GPIO module
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;

    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Configure the IO matrix
    ioCfg.isDQ32Bit = CyTrue;
    ioCfg.useUart = CyTrue;
    ioCfg.useI2C = CyFalse;
    ioCfg.useI2S = CyFalse;
    ioCfg.useSpi = CyFalse;
    ioCfg.lppMode = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    ioCfg.gpioSimpleEn[0] = 0;
    ioCfg.gpioSimpleEn[1] = (1 << (FREESRP_GPIO_LED - 32)) | (1 << (FREESRP_FPGA_DONE - 32));
    ioCfg.gpioComplexEn[0] = 0;
    ioCfg.gpioComplexEn[1] = 0;

    apiRetStatus = CyU3PDeviceConfigureIOMatrix(&ioCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        // Fatal error, cannot recover
        while(1);
    }

    // Configure the indicator LED GPIO
    gpioConfigLed.intrMode = CY_U3P_GPIO_NO_INTR;
    gpioConfigLed.outValue = CyFalse;
    gpioConfigLed.driveLowEn = CyTrue;
    gpioConfigLed.driveHighEn = CyTrue;
    gpioConfigLed.inputEn = CyFalse;

    apiRetStatus = CyU3PGpioSetSimpleConfig(FREESRP_GPIO_LED, &gpioConfigLed);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Configure the FPGA DONE input GPIO
    gpioConfigDone.intrMode = CY_U3P_GPIO_NO_INTR;
    gpioConfigDone.outValue = CyFalse;
    gpioConfigDone.driveLowEn = CyFalse;
    gpioConfigDone.driveHighEn = CyFalse;
    gpioConfigDone.inputEn = CyTrue;

    apiRetStatus = CyU3PGpioSetSimpleConfig(FREESRP_FPGA_DONE, &gpioConfigDone);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }
}

void SlaveFifo::stop()
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    /* Update the flag. */
    active = CyFalse;

    /* Disable the debug log mechanism. */
    //CyU3PDebugDeInit();

    /* Flush the endpoint memory */
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_CMD_IN);
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_CMD_OUT);
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_RX_DATA_IN);
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_TX_DATA_OUT);

    /* Destroy the DMA channel */
    CyU3PDmaChannelDestroy(&_dma_handle_uart_to_usb);
    CyU3PDmaChannelDestroy(&_dma_handle_usb_to_uart);
    CyU3PDmaChannelDestroy(&_dma_handle_gpif_to_usb);
    CyU3PDmaChannelDestroy(&_dma_handle_usb_to_gpif);

    /* Disable endpoints */
    CyU3PMemSet((uint8_t *)&epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyFalse;

    /* Endpoint configuration. */
    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_CMD_IN, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_CMD_OUT, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_TX_DATA_OUT, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_RX_DATA_IN, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 1);
    }
}

void SlaveFifo::setup_uart()
{
    CyU3PUartConfig_t uartConfig;
    CyU3PReturnStatus_t apiRetStatus;

    apiRetStatus = CyU3PUartInit();
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    CyU3PMemSet((uint8_t *)&uartConfig, 0, sizeof (uartConfig));
    uartConfig.baudRate = CY_U3P_UART_BAUDRATE_115200;
    uartConfig.stopBit = CY_U3P_UART_ONE_STOP_BIT;
    uartConfig.parity = CY_U3P_UART_NO_PARITY;
    uartConfig.txEnable = CyTrue;
    uartConfig.rxEnable = CyTrue;
    uartConfig.flowCtrl = CyFalse;
    uartConfig.isDma = CyTrue;

    apiRetStatus = CyU3PUartSetConfig(&uartConfig, NULL);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    apiRetStatus = CyU3PUartRxSetBlockXfer(0xFFFFFFFFU);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    apiRetStatus = CyU3PUartTxSetBlockXfer(0xFFFFFFFFU);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 3);
    }
}

void SlaveFifo::init_gpif()
{
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PPibClock_t pibClock;

    // Initialize the GPIF block
    pibClock.clkDiv = 2;
    pibClock.clkSrc = CY_U3P_SYS_CLK;
    pibClock.isHalfDiv = CyFalse;
    // Disable DLL for sync GPIF
    pibClock.isDllEnable = CyFalse;
    apiRetStatus = CyU3PPibInit(CyTrue, &pibClock);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    // Load the GPIF configuration for Slave FIFO sync mode
    apiRetStatus = CyU3PGpifLoad(&CyFxGpifConfig);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    // Configure GPIF sockets
    CyU3PGpifSocketConfigure(0, FREESRP_GPIF_SOCKET_RX_DATA, 6, CyFalse, 1);
    CyU3PGpifSocketConfigure(3, FREESRP_GPIF_SOCKET_TX_DATA, 6, CyFalse, 1);

    // Start GPIF state machine
    apiRetStatus = CyU3PGpifSMStart(RESET, ALPHA_RESET);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    // Register callback for errors
    CyU3PPibRegisterCallback(SlaveFifo::handle_gpif_error, CYU3P_PIB_INTR_ERROR);
}

void SlaveFifo::handle_gpif_error(CyU3PPibIntrType cbType, uint16_t cbArg)
{
    if(cbType == CYU3P_PIB_INTR_ERROR)
    {
        //uint8_t error = CYU3P_GET_PIB_ERROR_TYPE(cbArg);
        //CyFxAppErrorHandler(0, 7 + error);
        CyU3PGpioSetValue(FREESRP_GPIO_LED, 1);
        CyU3PThreadSleep(5);
        CyU3PGpioSetValue(FREESRP_GPIO_LED, 0);
    }
}