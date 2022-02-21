#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32/usb/core/usbd_cdc.h"

#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000

/** CDC Interface callback. */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(uint8_t* buf, uint16_t len);

/* GRBL USB receive callback */
void serial_receive_data(uint8_t* dataIn, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif