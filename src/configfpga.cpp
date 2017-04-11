#include "configfpga.h"
#include "main.h"

#include <cyu3system.h>
#include <cyu3error.h>
#include <cyu3usb.h>
#include <cyu3lpp.h>
#include <cyu3gpio.h>
#include <cyu3spi.h>

ConfigFpga::ConfigFpga() : active(false),
                           _config_done(false),
                           _packet_size(0),
                           _endpoint_seq_num(0)
{}

void ConfigFpga::init()
{
    CyU3PIoMatrixConfig_t io_cfg;
    CyU3PGpioClock_t gpioClock;
    CyU3PGpioSimpleConfig_t gpioConfig;
    CyU3PSpiConfig_t spiConfig;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    // Setup GPIOs
    io_cfg.useUart   = CyTrue;
    io_cfg.useI2C    = CyFalse;
    io_cfg.useI2S    = CyFalse;
    io_cfg.useSpi    = CyTrue;
    io_cfg.isDQ32Bit = CyFalse;
    io_cfg.lppMode   = CY_U3P_IO_MATRIX_LPP_DEFAULT;
    io_cfg.gpioSimpleEn[0]  = 0x00000000;
    io_cfg.gpioSimpleEn[1]  = (1 << (FREESRP_GPIO_LED - 32)) | (1 << (FREESRP_FPGA_INIT_B - 32)) | (1 << (FREESRP_FPGA_PROGRAM_B - 32)) | (1 << (FREESRP_FPGA_DONE - 32));
    io_cfg.gpioComplexEn[0] = 0;
    io_cfg.gpioComplexEn[1] = 0;
    apiRetStatus = CyU3PDeviceConfigureIOMatrix(&io_cfg);
    if(apiRetStatus != 0)
    {
        CyU3PDebugPrint(4, "IO matrix configuration failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Start the SPI module and configure the master.
    apiRetStatus = CyU3PSpiInit();
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "SPI init failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Start the SPI master block. Run the SPI clock at 25MHz
    // and configure the word length to 8 bits. Also configure
    // the slave select using FW.
    CyU3PMemSet((uint8_t *) &spiConfig, 0, sizeof(spiConfig));
    spiConfig.isLsbFirst = CyFalse;
    spiConfig.cpol       = CyFalse;
    spiConfig.ssnPol     = CyFalse;
    spiConfig.cpha       = CyFalse;
    spiConfig.leadTime   = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.lagTime    = CY_U3P_SPI_SSN_LAG_LEAD_HALF_CLK;
    spiConfig.ssnCtrl    = CY_U3P_SPI_SSN_CTRL_FW;
    spiConfig.clock      = 25000000;
    spiConfig.wordLen    = 8;

    apiRetStatus = CyU3PSpiSetConfig (&spiConfig, NULL);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "SPI config failed, Error Code = %d\n",apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Init the GPIO module
    gpioClock.fastClkDiv = 2;
    gpioClock.slowClkDiv = 0;
    gpioClock.simpleDiv = CY_U3P_GPIO_SIMPLE_DIV_BY_2;
    gpioClock.clkSrc = CY_U3P_SYS_CLK;
    gpioClock.halfDiv = 0;

    // Initialize GPIO interface
    apiRetStatus = CyU3PGpioInit(&gpioClock, NULL);
    if(apiRetStatus != 0)
    {
        CyU3PDebugPrint (4, "GPIO Init failed, Error Code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Configure INIT_B GPIO as input
    gpioConfig.outValue = CyFalse;
    gpioConfig.inputEn = CyTrue;
    gpioConfig.driveLowEn = CyFalse;
    gpioConfig.driveHighEn = CyFalse;
    gpioConfig.intrMode = CY_U3P_GPIO_INTR_BOTH_EDGE;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FREESRP_FPGA_INIT_B, &gpioConfig);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Configure DONE GPIO as input with interrupt enabled for both edges
    apiRetStatus = CyU3PGpioSetSimpleConfig(FREESRP_FPGA_DONE, &gpioConfig);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    // Configure PROGRAM_B output
    gpioConfig.outValue = CyTrue;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FREESRP_FPGA_PROGRAM_B, &gpioConfig);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }

    gpioConfig.outValue = CyFalse;
    gpioConfig.inputEn = CyFalse;
    gpioConfig.driveLowEn = CyTrue;
    gpioConfig.driveHighEn = CyTrue;
    gpioConfig.intrMode = CY_U3P_GPIO_NO_INTR;
    apiRetStatus = CyU3PGpioSetSimpleConfig(FREESRP_GPIO_LED, &gpioConfig);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PGpioSetSimpleConfig failed, error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 3);
    }
}

void ConfigFpga::start()
{
    uint16_t size = 0;
    CyU3PEpConfig_t epCfg;
    CyU3PDmaChannelConfig_t dmaCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;
    CyU3PUSBSpeed_t usbSpeed = CyU3PUsbGetSpeed();

    // First identify the usb speed. Once that is identified,
    // create a DMA channel and start the transfer on this.

    // Based on the Bus Speed configure the endpoint packet size
    switch (usbSpeed)
    {
    case CY_U3P_FULL_SPEED:
        size = 64;
        break;

    case CY_U3P_HIGH_SPEED:
        size = 512;
        break;

    case  CY_U3P_SUPER_SPEED:
        size = 1024;
        break;

    default:
        CyU3PDebugPrint(4, "Error! Invalid USB speed.\n");
        CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 1);
        break;
    }

    CyU3PMemSet((uint8_t *) &epCfg, 0, sizeof (epCfg));
    epCfg.enable = CyTrue;
    epCfg.epType = CY_U3P_USB_EP_BULK;
    epCfg.burstLen = 1;
    epCfg.streams = 0;
    epCfg.pcktSize = size;

    _packet_size = size;

    // Producer endpoint configuration
    apiRetStatus = CyU3PSetEpConfig(FREESRP_USB_ENDPOINT_TX_DATA_OUT, &epCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "CyU3PSetEpConfig failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 1);
    }

    // Create a DMA MANUAL channel for U2CPU transfer.
    // DMA size is set based on the USB speed.
    dmaCfg.size  = size;
    dmaCfg.count = 16;
    dmaCfg.prodSckId = FREESRP_USB_SOCKET_TX_DATA_OUT;
    dmaCfg.consSckId = CY_U3P_CPU_SOCKET_CONS;
    dmaCfg.dmaMode = CY_U3P_DMA_MODE_BYTE;
    // No callbacks
    dmaCfg.notification = 0;
    dmaCfg.cb = NULL;
    dmaCfg.prodHeader = 0;
    dmaCfg.prodFooter = 0;
    dmaCfg.consHeader = 0;
    dmaCfg.prodAvailCount = 0;

    apiRetStatus = CyU3PDmaChannelCreate(&_dma_handle_u_to_cpu, CY_U3P_DMA_TYPE_MANUAL_IN, &dmaCfg);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint (4, "CyU3PDmaChannelCreate failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    // Flush the Endpoint memory
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_TX_DATA_OUT);

    // Set DMA channel transfer size.
    apiRetStatus = CyU3PDmaChannelSetXfer(&_dma_handle_u_to_cpu, 0);
    if(apiRetStatus != CY_U3P_SUCCESS)
    {
        CyU3PDebugPrint(4, "CyU3PDmaChannelSetXfer Failed, Error code = %d\n", apiRetStatus);
        CyFxAppErrorHandler(apiRetStatus, 2);
    }

    // Update the status flag.
    active = true;
}

void ConfigFpga::stop()
{
    CyU3PEpConfig_t epCfg;
    CyU3PReturnStatus_t apiRetStatus = CY_U3P_SUCCESS;

    CyU3PGpioDeInit();
    CyU3PSpiDeInit();

    // Update flag
    active = false;

    // Get sequence number so the endpoint can be restored later after reinitializing it
    CyU3PUsbGetEpSeqNum(FREESRP_USB_ENDPOINT_TX_DATA_OUT, &_endpoint_seq_num);

    // Flush the endpoint memory
    CyU3PUsbFlushEp(FREESRP_USB_ENDPOINT_TX_DATA_OUT);

    // Destroy the channel
    CyU3PDmaChannelDestroy(&_dma_handle_u_to_cpu);
}

CyU3PReturnStatus_t ConfigFpga::config_fpga(uint32_t uiLen)
{
    uint32_t uiIdx;

    CyU3PReturnStatus_t apiRetStatus;
    CyU3PDmaBuffer_t inBuf_p;
    CyBool_t xFpga_Done, xFpga_Init_B;

    CyU3PDebugPrint(6, "file length: %d\n", uiLen);

    // Pull PROG_B line to reset FPGA
    apiRetStatus = CyU3PGpioSetValue(FREESRP_FPGA_PROGRAM_B, 0);
    CyU3PGpioSimpleGetValue(FREESRP_FPGA_INIT_B, &xFpga_Init_B);
    if(xFpga_Init_B)
    {
        // INIT_B not LOW
        CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 4);
        _config_done = CyFalse;
        return apiRetStatus;
    }

    CyU3PThreadSleep(10);

    // Release PROG_B line
    apiRetStatus = CyU3PGpioSetValue(FREESRP_FPGA_PROGRAM_B, 1);
    CyU3PThreadSleep(10); // Allow FPGA to startup

    // Check if FPGA is now ready by testing the INIT_B signal
    apiRetStatus |= CyU3PGpioSimpleGetValue(FREESRP_FPGA_INIT_B, &xFpga_Init_B);
    if((xFpga_Init_B != CyTrue) || (apiRetStatus != CY_U3P_SUCCESS))
    {
        CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 5);
        return apiRetStatus;
    }

    // Start shifting out configuration data
    for(uiIdx = 0; (uiIdx < uiLen) && active; uiIdx += _packet_size)
    {
        if(CyU3PDmaChannelGetBuffer(&_dma_handle_u_to_cpu, &inBuf_p, 2000) != CY_U3P_SUCCESS)
        {
            // Wait 2000 ms(?)
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 7);
            _config_done = CyFalse;
            apiRetStatus = CY_U3P_ERROR_TIMEOUT;
            break;
        }

        apiRetStatus = CyU3PSpiTransmitWords(inBuf_p.buffer, _packet_size);

        if(apiRetStatus != CY_U3P_SUCCESS)
        {
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 8);
            _config_done = CyFalse;
            break;
        }

        if(CyU3PDmaChannelDiscardBuffer(&_dma_handle_u_to_cpu) != CY_U3P_SUCCESS)
        {
            CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 9);
            _config_done = CyFalse;
            apiRetStatus = CY_U3P_ERROR_TIMEOUT;
            break;
        }
    }

    CyU3PThreadSleep(1);

    apiRetStatus |= CyU3PGpioSimpleGetValue(FREESRP_FPGA_DONE, &xFpga_Done);
    if((xFpga_Done != CyTrue))
    {
        CyFxAppErrorHandler(CY_U3P_ERROR_FAILURE, 6);
        _config_done = CyFalse;
        apiRetStatus = CY_U3P_ERROR_FAILURE;
    }
    else
    {
        _config_done = CyTrue;
    }

    return apiRetStatus;
}

CyBool_t ConfigFpga::config_done()
{
    return _config_done;
}

uint8_t ConfigFpga::endpoint_seq_num()
{
    return _endpoint_seq_num;
}