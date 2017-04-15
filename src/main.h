#ifndef FREESRP_MAIN_H
#define FREESRP_MAIN_H

#include <cyu3types.h>
#include <cyu3usbconst.h>
#include <cyu3externcstart.h>

#define FREESRP_VERSION "0.1.1"

#define CY_FX_APPLN_THREAD_STACK       (0x1000)                  /* Debug application thread stack size */
#define CY_FX_APPLN_THREAD_PRIORITY    (8)                       /* Debug application thread priority */

/* Endpoint and socket definitions for the application */
#define FREESRP_USB_ENDPOINT_CMD_OUT		0x01    /* EP 1 OUT */
#define FREESRP_USB_ENDPOINT_CMD_IN         0x81    /* EP 1 IN */
#define FREESRP_USB_ENDPOINT_TX_DATA_OUT	0x02	/* EP 2 OUT */
#define FREESRP_USB_ENDPOINT_RX_DATA_IN		0x82	/* EP 2 IN */

#define FREESRP_USB_SOCKET_CMD_OUT          CY_U3P_UIB_SOCKET_PROD_1    /* USB Socket 1 is producer */
#define FREESRP_USB_SOCKET_CMD_IN           CY_U3P_UIB_SOCKET_CONS_1    /* USB Socket 1 is consumer */
#define FREESRP_USB_SOCKET_TX_DATA_OUT      CY_U3P_UIB_SOCKET_PROD_2	/* USB Socket 2 is producer */
#define FREESRP_USB_SOCKET_RX_DATA_IN       CY_U3P_UIB_SOCKET_CONS_2	/* USB Socket 2 is consumer */

/* GPIF II P-Port sockets */
#define FREESRP_GPIF_SOCKET_RX_DATA         CY_U3P_PIB_SOCKET_0			/* P-Port Socket 0 is producer */
#define FREESRP_GPIF_SOCKET_TX_DATA         CY_U3P_PIB_SOCKET_3			/* P-Port Socket 3 is consumer */

/* Vendor commands */
#define FREESRP_VENDOR_GET_VERSION          0
#define FREESRP_VENDOR_FPGA_CONFIG_LOAD     0xB2
#define FREESRP_VENDOR_FPGA_CONFIG_STATUS   0xB1
#define FREESRP_VENDOR_FPGA_CONFIG_FINISH   0xB3
#define FREESRP_VENDOR_XCVR_INIT            0xC0
#define FREESRP_VENDOR_XCVR_CMD             0xC1

/* Events */
#define FREESRP_EVENT_FPGA_CONFIG_START     (1 << 0)   	/* event to initiate FPGA configuration */
#define FREESRP_EVENT_FPGA_CONFIG_FINISH    (1 << 1)   	/* event to initiate switch back to slave FIFO */

/* GPIOs */
#define FREESRP_GPIO_LED 51

#define FREESRP_FPGA_INIT_B 50
#define FREESRP_FPGA_DONE 57
#define FREESRP_FPGA_PROGRAM_B 52

/* Misc */
#define FREESRP_UART_DMA_BUF_SIZE 32

/* Extern definitions for the USB Descriptors */
extern const uint8_t CyFxUSB20DeviceDscr[];
extern const uint8_t CyFxUSB30DeviceDscr[];
extern const uint8_t CyFxUSBDeviceQualDscr[];
extern const uint8_t CyFxUSBFSConfigDscr[];
extern const uint8_t CyFxUSBHSConfigDscr[];
extern const uint8_t CyFxUSBBOSDscr[];
extern const uint8_t CyFxUSBSSConfigDscr[];
extern const uint8_t CyFxUSBStringLangIDDscr[];
extern const uint8_t CyFxUSBManufactureDscr[];
extern const uint8_t CyFxUSBProductDscr[];

extern void CyFxAppErrorHandler(CyU3PReturnStatus_t apiRetStatus, uint8_t blinkCode);

#include "cyu3externcend.h"

#endif
