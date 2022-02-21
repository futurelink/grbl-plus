#include "stm32/usb/usbd_cdc_if.h"

// Serial baud rate
#define BAUD_RATE 115200

uint8_t lcBuffer[7]; // Line coding buffer
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

extern USBD_HandleTypeDef hUsbDeviceFS;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

static int8_t CDC_Init_FS(void) {
    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

    uint32_t baudRate = BAUD_RATE;
    lcBuffer[0] = (uint8_t)(baudRate);
    lcBuffer[1] = (uint8_t)(baudRate >> 8);
    lcBuffer[2] = (uint8_t)(baudRate >> 16);
    lcBuffer[3] = (uint8_t)(baudRate >> 24);
    lcBuffer[4] = 0; // 1 Stop bit
    lcBuffer[5] = 0; // No parity
    lcBuffer[6] = 8; // 8 data bits

    return USBD_OK;
}

static int8_t CDC_DeInit_FS(void) {
    return USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length){
    switch(cmd) {
        case CDC_SEND_ENCAPSULATED_COMMAND: break;
        case CDC_GET_ENCAPSULATED_RESPONSE: break;
        case CDC_SET_COMM_FEATURE: break;
        case CDC_GET_COMM_FEATURE: break;
        case CDC_CLEAR_COMM_FEATURE: break;

        /*******************************************************************************/
        /* Line Coding Structure                                                       */
        /*-----------------------------------------------------------------------------*/
        /* Offset | Field       | Size | Value  | Description                          */
        /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
        /* 4      | bCharFormat |   1  | Number | Stop bits                            */
        /*                                        0 - 1 Stop bit                       */
        /*                                        1 - 1.5 Stop bits                    */
        /*                                        2 - 2 Stop bits                      */
        /* 5      | bParityType |  1   | Number | Parity                               */
        /*                                        0 - None                             */
        /*                                        1 - Odd                              */
        /*                                        2 - Even                             */
        /*                                        3 - Mark                             */
        /*                                        4 - Space                            */
        /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
        /*******************************************************************************/
        case CDC_SET_LINE_CODING:
            lcBuffer[0] = pbuf[0];
            lcBuffer[1] = pbuf[1];
            lcBuffer[2] = pbuf[2];
            lcBuffer[3] = pbuf[3];
            lcBuffer[4] = pbuf[4];
            lcBuffer[5] = pbuf[5];
            lcBuffer[6] = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = lcBuffer[0];
            pbuf[1] = lcBuffer[1];
            pbuf[2] = lcBuffer[2];
            pbuf[3] = lcBuffer[3];
            pbuf[4] = lcBuffer[4];
            pbuf[5] = lcBuffer[5];
            pbuf[6] = lcBuffer[6];

            // Get line coding is invoked when the host connects, clear the RxBuffer when this occurs
            //CDC_FlushRxBuffer_FS();
            break;

        case CDC_SET_CONTROL_LINE_STATE: break;
        case CDC_SEND_BREAK: break;
        default: break;
    }

    return USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) {
    uint8_t len = *Len;
    OnUsbDataRx(Buf, len);

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    return USBD_OK;
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len) {
    uint8_t result;

    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0) {
        return USBD_BUSY;
    }

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
    result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

    return result;
}
