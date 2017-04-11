#ifndef FREESRP_CONFIGFPGA_H
#define FREESRP_CONFIGFPGA_H

#include <cyu3types.h>
#include <cyu3usbconst.h>
#include <cyu3dma.h>

class ConfigFpga
{
public:
    ConfigFpga();

    /*
     * Initializes the GPIOs and SPI needed for FPGA configuration.
     * You should enumerate after this.
     */
    void init();

    /*
     * Configure USB endpoint and DMA channel. Ready to config_fpga() after this.
     */
    void start();

    /*
     * Deinitialize GPIO and SPI and destroy DMA channel and endpoint.
     */
    void stop();

    /*
     * Configure the FPGA.
     * uiLen: config file length
     */
    CyU3PReturnStatus_t config_fpga(uint32_t uiLen);

    /*
     * Check if config done.
     */
    CyBool_t config_done();

    /*
     * Get the endpoint sequence number so you can restore it when reconfiguring the endpoint in the main app
     */
    uint8_t endpoint_seq_num(); // TODO: need to restore sequence number after reconfiguring endpoint in main app
public:
    volatile CyBool_t active;
private:
    CyBool_t _config_done;
    CyU3PDmaChannel _dma_handle_u_to_cpu;
    uint16_t _packet_size;
    uint8_t _endpoint_seq_num;
};

#endif
