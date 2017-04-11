#ifndef FREESRP_SLAVE_FIFO_H
#define FREESRP_SLAVE_FIFO_H

#include <cyu3types.h>
#include <cyu3usbconst.h>
#include <cyu3dma.h>
#include <cyu3pib.h>

class SlaveFifo
{
public:
    SlaveFifo();

    /*
     * Initializes GPIOs, UART and GPIF
     */
    void init();

    /*
     * Configure USB endpoints and DMA channels
     */
    void start();

    /*
     * Destroy DMA channels and endpoints
     */
    void stop();
private:
    void init_gpio();
    void setup_uart();
    void init_gpif();

    static void handle_gpif_error(CyU3PPibIntrType cbType, uint16_t cbArg);
public:
    volatile CyBool_t active;

    CyU3PDmaChannel _dma_handle_uart_to_usb;
    CyU3PDmaChannel _dma_handle_usb_to_uart;

    CyU3PDmaChannel _dma_handle_usb_to_gpif;
    CyU3PDmaChannel _dma_handle_gpif_to_usb;
};

#endif